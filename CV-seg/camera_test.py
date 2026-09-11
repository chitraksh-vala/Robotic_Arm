import cv2

for i in range(10):
    cap = cv2.VideoCapture(i)

    if cap.isOpened():
        ret, frame = cap.read()

        if ret:
            print(f"Camera {i}: FOUND - {frame.shape[1]}x{frame.shape[0]}")
        else:
            print(f"Camera {i}: detected but couldn't read")

        cap.release()
    else:
        print(f"Camera {i}: not found")