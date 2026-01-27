#!/usr/bin/env python3
"""Tiny OpenCV script to preview the laptop webcam."""

import cv2


def main() -> None:
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        raise RuntimeError("Unable to open webcam (index 0)")

    while True:
        ok, frame = cap.read()
        if not ok:
            break
        cv2.imshow("webcam_preview", frame)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
