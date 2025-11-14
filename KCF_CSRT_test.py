import cv2


def create_tracker(type="KCF"):
    if type == "CSRT":
        return cv2.TrackerCSRT_create()
    else:
        return cv2.TrackerKCF_create()


def draw_bbox(frame):
    box, start, drawing = [0, 0, 0, 0], None, False

    def mouse(event, x, y, flags, param):
        nonlocal box, start, drawing
        tmp = frame.copy()
        if event == cv2.EVENT_LBUTTONDOWN:
            start, drawing = (x, y), True
        elif event == cv2.EVENT_MOUSEMOVE and drawing:
            cv2.rectangle(tmp, start, (x, y), (0, 255, 0), 2)
            cv2.imshow("ROI", tmp)
        elif event == cv2.EVENT_LBUTTONUP:
            drawing = False
            box = (
                min(start[0], x),
                min(start[1], y),
                abs(x - start[0]),
                abs(y - start[1]),
            )
            cv2.rectangle(tmp, start, (x, y), (0, 255, 0), 2)
            cv2.imshow("ROI", tmp)

    cv2.namedWindow("ROI")
    cv2.setMouseCallback("ROI", mouse)
    cv2.imshow("ROI", frame)
    print("Draw ROI, release to confirm")
    while cv2.waitKey(10) & 0xFF != 13 and box == [ 0, 0, 0, 0]:  # Enter to confirm
        pass
    cv2.destroyWindow("ROI")
    return box


cap = cv2.VideoCapture(0)
if not cap.isOpened():
    raise RuntimeError("Camera not found")

ret, frame = cap.read()
tracker = create_tracker("CSRT")
bbox = draw_bbox(frame)
tracker.init(frame, bbox)

while True:
    ret, frame = cap.read()
    if not ret:
        break
    ok, box = tracker.update(frame)
    if ok:
        (x, y, w, h) = map(int, box)
        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
        cv2.putText(frame, "Tracking", (10, 30), 1, 1, (0, 255, 0), 2)
    else:
        cv2.putText(frame, "Lost", (10, 30), 1, 1, (0, 0, 255), 2)
    cv2.putText(
        frame,
        "'r' to redraw, 'ESC' to quit",
        (10, frame.shape[0] - 10),
        1,
        0.7,
        (255, 255, 255),
        1,
    )
    cv2.imshow("KCF Tracker", frame)

    k = cv2.waitKey(1) & 0xFF
    if k == 27:
        break
    elif k == ord("r"):
        bbox = draw_bbox(frame)
        tracker = create_tracker("CSRT")
        tracker.init(frame, bbox)

cap.release()
cv2.destroyAllWindows()
