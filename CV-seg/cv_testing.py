import cv2
import numpy as np

# =============================
# CAMERA SETUP
# =============================

cap = cv2.VideoCapture(1, cv2.CAP_DSHOW)

cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

if not cap.isOpened():
    print("ERROR: Could not open external camera.")
    exit()

print("Camera started.")
print("Press Q to quit.")


# =============================
# MAIN LOOP
# =============================

while True:

    ret, frame = cap.read()

    if not ret:
        print("ERROR: Could not read frame.")
        break

    # -----------------------------
    # Convert BGR → HSV
    # -----------------------------

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)


    # =============================
    # RED COLOR DETECTION
    # =============================

    # Lower red range
    lower_red1 = np.array([0, 100, 100])
    upper_red1 = np.array([10, 255, 255])

    # Upper red range
    lower_red2 = np.array([170, 100, 100])
    upper_red2 = np.array([180, 255, 255])

    mask1 = cv2.inRange(
        hsv,
        lower_red1,
        upper_red1
    )

    mask2 = cv2.inRange(
        hsv,
        lower_red2,
        upper_red2
    )

    # Combine both red ranges
    mask = mask1 | mask2


    # =============================
    # REMOVE NOISE
    # =============================

    kernel = np.ones((5, 5), np.uint8)

    # Remove small isolated regions
    mask = cv2.morphologyEx(
        mask,
        cv2.MORPH_OPEN,
        kernel
    )

    # Fill small holes
    mask = cv2.morphologyEx(
        mask,
        cv2.MORPH_CLOSE,
        kernel
    )


    # =============================
    # FIND CONTOURS
    # =============================

    contours, _ = cv2.findContours(
        mask,
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE
    )


    # =============================
    # FIND LARGEST OBJECT
    # =============================

    if contours:

        largest = max(
            contours,
            key=cv2.contourArea
        )

        area = cv2.contourArea(largest)


        # Ignore small noise
        if area > 500:

            # -----------------------------
            # Bounding box
            # -----------------------------

            x, y, w, h = cv2.boundingRect(largest)


            # -----------------------------
            # Contour centroid
            # -----------------------------

            M = cv2.moments(largest)

            if M["m00"] != 0:

                center_x = int(M["m10"] / M["m00"])
                center_y = int(M["m01"] / M["m00"])


                # =============================
                # DRAW OBJECT
                # =============================

                # Bounding box
                cv2.rectangle(
                    frame,
                    (x, y),
                    (x + w, y + h),
                    (0, 255, 0),
                    2
                )

                # Centroid
                cv2.circle(
                    frame,
                    (center_x, center_y),
                    6,
                    (0, 0, 255),
                    -1
                )


                # =============================
                # DISPLAY INFORMATION
                # =============================

                cv2.putText(
                    frame,
                    f"Center: ({center_x}, {center_y})",
                    (x, y - 10),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (0, 255, 0),
                    2
                )

                cv2.putText(
                    frame,
                    f"Area: {int(area)}",
                    (x, y + h + 25),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (0, 255, 0),
                    2
                )


                # Print coordinates
                print(
                    f"Object center: "
                    f"X={center_x}, Y={center_y}"
                )


    # =============================
    # DISPLAY
    # =============================

    cv2.imshow(
        "Robot Vision",
        frame
    )

    cv2.imshow(
        "Mask",
        mask
    )


    # =============================
    # QUIT
    # =============================

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break


# =============================
# CLEANUP
# =============================

cap.release()
cv2.destroyAllWindows()