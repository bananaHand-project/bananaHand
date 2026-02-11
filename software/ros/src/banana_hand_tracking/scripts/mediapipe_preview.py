#!/usr/bin/env python3
"""MediaPipe Hands preview that prints when a hand is detected."""

import sys

import cv2

try:
    import mediapipe as mp
except ImportError as exc:
    raise SystemExit(
        "mediapipe is not installed. Try: pip install mediapipe"
    ) from exc


def main() -> None:
    cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
    if not cap.isOpened():
        raise RuntimeError("Unable to open webcam (index 0)")

    mp_hands = mp.solutions.hands
    mp_draw = mp.solutions.drawing_utils
    hands = mp_hands.Hands(
        static_image_mode=False,
        max_num_hands=1,
        min_detection_confidence=0.5,
        min_tracking_confidence=0.5,
    )

    while True:
        ok, frame = cap.read()
        if not ok:
            continue

        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = hands.process(rgb)

        if results.multi_hand_landmarks:
            print("hand detected")
            for landmarks in results.multi_hand_landmarks:
                mp_draw.draw_landmarks(
                    frame, landmarks, mp_hands.HAND_CONNECTIONS
                )

        cv2.imshow("mediapipe_hands", frame)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    hands.close()
    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        sys.exit(0)
