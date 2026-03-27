#!/usr/bin/env python3
"""
extract_frames.py
-----------------
Extrae frames de un topic de cámara (comprimido o raw)
de un rosbag2 y los guarda como .jpg en una carpeta de salida.

Uso:
    python3 extract_frames.py --bag /path/to/bag --output /path/to/frames
    python3 extract_frames.py --bag /path/to/bag --output /path/to/frames --every 3
"""

import argparse
import os
import sys
from pathlib import Path

import cv2
import numpy as np


ZSTD_MAGIC = b"\x28\xb5\x2f\xfd"


def _maybe_decompress_zstd(raw_data: bytes):
    if not raw_data.startswith(ZSTD_MAGIC):
        return raw_data, False

    try:
        import zstandard as zstd
    except ImportError:
        print(
            "[ERROR] El mensaje del bag está comprimido con zstd, "
            "pero falta el módulo Python 'zstandard'."
        )
        print("        Instalación sugerida: pip install zstandard")
        sys.exit(1)

    try:
        dctx = zstd.ZstdDecompressor()
        return dctx.decompress(raw_data), True
    except Exception as e:
        print(f"[ERROR] No se pudo descomprimir payload zstd: {e}")
        return raw_data, False


def _decode_frame_from_serialized(raw_data: bytes):
    """
    Intenta recuperar una imagen comprimida directamente desde bytes serializados.
    Útil cuando `deserialize_message` falla por incompatibilidades CDR.
    """
    # 1) Caso ideal: el payload completo ya es JPEG/PNG.
    frame = cv2.imdecode(np.frombuffer(raw_data, dtype=np.uint8), cv2.IMREAD_COLOR)
    if frame is not None:
        return frame

    # 2) Fallback JPEG: recortar entre SOI y EOI para evitar bytes extra.
    jpeg_soi = raw_data.find(b"\xff\xd8")
    if jpeg_soi != -1:
        jpeg_eoi = raw_data.find(b"\xff\xd9", jpeg_soi + 2)
        if jpeg_eoi != -1:
            jpeg_bytes = raw_data[jpeg_soi:jpeg_eoi + 2]
            frame = cv2.imdecode(np.frombuffer(jpeg_bytes, dtype=np.uint8), cv2.IMREAD_COLOR)
            if frame is not None:
                return frame

    # 3) Fallback PNG: recortar entre signature e IEND.
    png_sig = b"\x89PNG\r\n\x1a\n"
    png_iend = b"IEND\xaeB`\x82"
    png_start = raw_data.find(png_sig)
    if png_start != -1:
        png_end = raw_data.find(png_iend, png_start + len(png_sig))
        if png_end != -1:
            png_bytes = raw_data[png_start:png_end + len(png_iend)]
            frame = cv2.imdecode(np.frombuffer(png_bytes, dtype=np.uint8), cv2.IMREAD_COLOR)
            if frame is not None:
                return frame

    return None


def _decode_raw_sensor_image(msg):
    """
    Convierte un sensor_msgs/msg/Image (raw) a imagen BGR (OpenCV).
    """
    encoding = (msg.encoding or "").lower()
    height = int(msg.height)
    width = int(msg.width)
    step = int(msg.step)

    if height <= 0 or width <= 0 or step <= 0:
        return None

    # Encodings de 8 bits por canal
    if encoding in {
        "bgr8", "rgb8", "bgra8", "rgba8", "mono8", "8uc1", "8uc3", "8uc4"
    }:
        channels_map = {
            "mono8": 1,
            "8uc1": 1,
            "bgr8": 3,
            "rgb8": 3,
            "8uc3": 3,
            "bgra8": 4,
            "rgba8": 4,
            "8uc4": 4,
        }
        channels = channels_map.get(encoding, 3)
        row_bytes = width * channels

        buf = np.frombuffer(msg.data, dtype=np.uint8)
        if buf.size < height * step:
            return None

        rows = buf[: height * step].reshape((height, step))
        if row_bytes > step:
            return None
        img = rows[:, :row_bytes].reshape((height, width, channels))

        if channels == 1:
            return cv2.cvtColor(img.reshape((height, width)), cv2.COLOR_GRAY2BGR)
        if encoding in {"rgb8"}:
            return cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        if encoding in {"rgba8"}:
            return cv2.cvtColor(img, cv2.COLOR_RGBA2BGR)
        if encoding in {"bgra8"}:
            return cv2.cvtColor(img, cv2.COLOR_BGRA2BGR)
        return img

    # Encodings de 16 bits (se normaliza a 8 bits para guardar JPG)
    if encoding in {"mono16", "16uc1"}:
        if step < width * 2:
            return None
        buf = np.frombuffer(msg.data, dtype=np.uint8)
        if buf.size < height * step:
            return None
        rows = buf[: height * step].reshape((height, step))[:, : width * 2]
        img16 = rows.reshape((height, width, 2)).view(np.uint16).reshape((height, width))
        img8 = cv2.convertScaleAbs(img16, alpha=(255.0 / 65535.0))
        return cv2.cvtColor(img8, cv2.COLOR_GRAY2BGR)

    return None


def extract_frames(bag_path: str, output_dir: str, every_n: int, topic: str):
    try:
        import rclpy
        from rclpy.serialization import deserialize_message
        from rosidl_runtime_py.utilities import get_message
        import rosbag2_py
    except ImportError as e:
        print(f"[ERROR] Dependencia ROS2 no encontrada: {e}")
        print("Asegurate de haber hecho source /opt/ros/humble/setup.bash")
        sys.exit(1)

    output_path = Path(output_dir)
    output_path.mkdir(parents=True, exist_ok=True)

    storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id="sqlite3")
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format="cdr",
        output_serialization_format="cdr",
    )

    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)

    topic_types = reader.get_all_topics_and_types()
    type_map = {t.name: t.type for t in topic_types}

    if topic not in type_map:
        available = [t.name for t in topic_types]
        print(f"[ERROR] Topic '{topic}' no encontrado en el bag.")
        print(f"Topics disponibles:\n  " + "\n  ".join(available))
        sys.exit(1)

    msg_type = get_message(type_map[topic])
    msg_type_name = type_map[topic]
    is_compressed_topic = msg_type_name == "sensor_msgs/msg/CompressedImage"
    is_raw_topic = msg_type_name == "sensor_msgs/msg/Image"

    filter_ = rosbag2_py.StorageFilter(topics=[topic])
    reader.set_filter(filter_)

    frame_count = 0
    saved_count = 0
    decode_fail_count = 0
    warned_fallback = False
    warned_zstd = False
    warned_unknown_encoding = False
    warned_raw_deserialize = False

    print(f"[INFO] Leyendo bag: {bag_path}")
    print(f"[INFO] Topic: {topic}")
    print(f"[INFO] Guardando cada {every_n} frame(s) en: {output_dir}")

    while reader.has_next():
        (topic_name, data, timestamp_ns) = reader.read_next()

        raw_data, was_zstd = _maybe_decompress_zstd(data)
        if was_zstd and not warned_zstd:
            print("[INFO] Detectada compresión zstd en payload de mensajes. Descomprimiendo...")
            warned_zstd = True

        if frame_count % every_n != 0:
            frame_count += 1
            continue

        frame = None

        # Camino principal: deserialización ROS2 estándar.
        try:
            msg = deserialize_message(raw_data, msg_type)
            if is_compressed_topic:
                np_arr = np.frombuffer(msg.data, dtype=np.uint8)
                frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            elif is_raw_topic:
                frame = _decode_raw_sensor_image(msg)
                if frame is None and not warned_unknown_encoding:
                    print(
                        f"[WARN] Encoding raw no soportado o inválido: '{msg.encoding}'."
                    )
                    warned_unknown_encoding = True
            else:
                # Heurística: si no es tipo conocido, intentar como comprimida.
                np_arr = np.frombuffer(msg.data, dtype=np.uint8)
                frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        except Exception:
            # Fallback para bags con incompatibilidades de CDR/XCDR.
            if is_compressed_topic:
                frame = _decode_frame_from_serialized(raw_data)
                if frame is not None and not warned_fallback:
                    print(
                        "[WARN] Falló la deserialización ROS2 estándar; "
                        "usando extracción directa del payload comprimido."
                    )
                    warned_fallback = True
            else:
                frame = None
                if not warned_raw_deserialize:
                    print(
                        "[WARN] Falló la deserialización ROS2 en topic raw; "
                        "no se puede aplicar fallback por bytes comprimidos."
                    )
                    warned_raw_deserialize = True

        if frame is None:
            print(f"[WARN] Frame {frame_count} no pudo decodificarse, saltando.")
            decode_fail_count += 1
            frame_count += 1
            continue

        # Nombre del archivo: timestamp en nanosegundos para orden cronológico exacto
        filename = output_path / f"frame_{timestamp_ns:020d}.jpg"
        cv2.imwrite(str(filename), frame, [cv2.IMWRITE_JPEG_QUALITY, 95])
        saved_count += 1

        if saved_count % 50 == 0:
            print(f"[INFO] Guardados {saved_count} frames...")

        frame_count += 1

    print(f"\n[OK] Extracción completa.")
    print(f"     Frames totales leídos : {frame_count}")
    print(f"     Frames guardados       : {saved_count}")
    print(f"     Frames no decodificados: {decode_fail_count}")
    print(f"     Directorio             : {output_dir}")

    expected_saved = max(1, frame_count // max(1, every_n))
    if frame_count > 0 and saved_count < max(5, int(expected_saved * 0.5)):
        print("[WARN] Se guardaron pocos frames respecto a los leídos.")
        print("       Revisá que el --topic sea correcto y el tipo/encoding de imagen sea compatible.")


def main():
    parser = argparse.ArgumentParser(
        description="Extrae frames de un rosbag2 desde un topic de imagen comprimida."
    )
    parser.add_argument(
        "--bag",
        required=True,
        help="Ruta al directorio del rosbag2 (carpeta que contiene metadata.yaml)",
    )
    parser.add_argument(
        "--output",
        required=True,
        help="Directorio donde se guardarán los frames .jpg",
    )
    parser.add_argument(
        "--every",
        type=int,
        default=1,
        help="Guardar 1 de cada N frames (default: 1, todos los frames)",
    )
    parser.add_argument(
        "--topic",
        default="/telemetry/camera_record/compressed",
        help="Topic de imagen comprimida a extraer",
    )
    args = parser.parse_args()

    if not os.path.isdir(args.bag):
        print(f"[ERROR] El path del bag no es un directorio válido: {args.bag}")
        sys.exit(1)

    if args.every < 1:
        print(f"[ERROR] --every debe ser >= 1 (valor recibido: {args.every})")
        sys.exit(1)

    extract_frames(
        bag_path=args.bag,
        output_dir=args.output,
        every_n=args.every,
        topic=args.topic,
    )


if __name__ == "__main__":
    main()