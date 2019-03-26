import numpy as np
import cv2

import nav.optic_intf as intf


handle = intf.StereoHandle(lambda: None)
handle.start()

try:
    while True:
        # Capture frame-by-frame
        frame = handle.read_frame()

        # Display the resulting frame
        cv2.imshow('frame', frame.arr)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    # When everything done, release the capture
    handle.stop()
    cv2.destroyAllWindows()
