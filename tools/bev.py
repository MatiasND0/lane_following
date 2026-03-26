import cv2
import numpy as np

# =============================
# BEV Calibration Tool
# =============================
# - Load image
# - Drag 4 source points
# - See BEV in real time
# - Export ready-to-use C++ config
# =============================

IMG_PATH = "/home/hhouse/ws/lineas_buenas/frames/frame_01771502939992035638.jpg"  # change if needed

# Load image
img = cv2.imread(IMG_PATH)
if img is None:
    raise Exception("Image not found. Check path.")

h, w = img.shape[:2]

# Initial points (trapecio)
src_pts = np.array([
    [int(w*0.4), int(h*0.55)],
    [int(w*0.6), int(h*0.55)],
    [int(w*0.95), int(h*0.95)],
    [int(w*0.05), int(h*0.95)]
], dtype=np.float32)

# Destination (rectangulo)
bev_w, bev_h = 320, 240

dst_pts = np.array([
    [int(bev_w*0.1), 0],
    [int(bev_w*0.9), 0],
    [int(bev_w*0.9), bev_h],
    [int(bev_w*0.1), bev_h]
], dtype=np.float32)

selected_point = -1

# Mouse callback

def mouse_callback(event, x, y, flags, param):
    global selected_point, src_pts

    if event == cv2.EVENT_LBUTTONDOWN:
        dists = np.linalg.norm(src_pts - np.array([x, y]), axis=1)
        selected_point = np.argmin(dists)

    elif event == cv2.EVENT_MOUSEMOVE and selected_point != -1:
        src_pts[selected_point] = [x, y]

    elif event == cv2.EVENT_LBUTTONUP:
        selected_point = -1

cv2.namedWindow("Original")
cv2.setMouseCallback("Original", mouse_callback)

while True:
    display = img.copy()

    # Draw points
    for i, pt in enumerate(src_pts):
        color = (0,255,0)
        cv2.circle(display, tuple(pt.astype(int)), 6, color, -1)
        cv2.putText(display, str(i), tuple(pt.astype(int)+5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 1)

    # Draw polygon
    pts_int = src_pts.astype(int)
    cv2.polylines(display, [pts_int], True, (255,0,0), 2)

    # Compute homography
    H = cv2.getPerspectiveTransform(src_pts, dst_pts)
    bev = cv2.warpPerspective(img, H, (bev_w, bev_h))

    # Show
    cv2.imshow("Original", display)
    cv2.imshow("BEV", bev)

    key = cv2.waitKey(1)

    if key == 27:  # ESC
        break

    elif key == ord('p'):
        print("\n--- Normalized (for understanding) ---")
        for pt in src_pts:
            print(f"{pt[0]/w:.3f}, {pt[1]/h:.3f}")

    elif key == ord('c'):
        print("\n--- COPY & PASTE into C++ ---")
        print("std::vector<double> src_pts = {")
        for pt in src_pts:
            print(f"    {pt[0]:.1f}, {pt[1]:.1f},")
        print("};")

cv2.destroyAllWindows()
