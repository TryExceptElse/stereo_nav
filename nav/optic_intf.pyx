#cython: language_level=3
"""
This module handles generation of numpy arrays from the stereo device.
"""

cimport cython
cimport uvc
from libc.stdint cimport uint8_t, uint16_t
import numpy as np
cimport numpy as np

import time

cimport uvc


#######################################################################
# Extern declarations.


cdef extern from 'clib/optic_intf.h':
    # Optic Types

    cdef enum optic_status:
        kOpticOk = 0
        kOpticDeviceNotFound
        kOpticPermissionError
        kOpticDeviceOpenError
        kOpticStreamControlError
        kOpticStreamStartError
        kOpticHandleStateError
        kOpticAllocFailure

    cdef enum optic_side:
        kOpticLeft
        kOpticRight

    cdef struct optic_stereo_handle_t

    cdef struct optic_cb_data_t:
        optic_side side
        uvc.uvc_frame_t *frame
        void *cb_data

    # Extern vars

    const uint16_t kOpticWidth  # Stream width
    const uint16_t kOpticHeight  # Stream height
    const uint16_t kOpticFps

    # Optic Functions

    optic_status optic_open_handle(
        uvc.uvc_context_t *ctx,
        optic_stereo_handle_t **handle,
        void (*cb)(optic_cb_data_t data),
        void *cb_data
    )

    void optic_close_handle(optic_stereo_handle_t *handle)

    optic_status optic_handle_start_streaming(optic_stereo_handle_t *handle)

    bint optic_handle_stop_streaming(optic_stereo_handle_t *handle)

    void optic_sprint_status(
        char *buf,
        uint16_t n,
        optic_status status
    )


#######################################################################


cdef double MAX_SYNC_DIFF = 1 / 60  # Half of a frame duration.


class OpticDeviceException(Exception):
    """
    Python Exception thrown when a UVC error occurs.
    """

cdef struct received_frames_t:
    pass  # TODO


cdef void optic_callback(optic_cb_data_t data):
    """
    This function is called by the optic interface when a frame is
    received from either sensor.
    """
    cdef received_frames_t *received_frames = <received_frames_t *>data.cb_data
    if data.side == kOpticLeft:
        pass # TODO

    if data.side == kOpticRight:
        pass # TODO


cdef void store_frame_in_arr(uvc.uvc_frame_t *frame, uint8_t *arr) nogil:
    pass  # TODO


@cython.final
cdef class StereoHandle:
    """
    Class handling Input from a stereo camera device.
    """

    cdef optic_stereo_handle_t *handle
    cdef optic_cb_data_t cb_data
    cdef uvc.uvc_context_t *uvc_context
    cdef object cb
    cdef received_frames_t received_frames

    # If someday multiple stereo handles need to be controlled,
    # they should be added as arguments to __init__.


    def __cinit__(self, cb) -> None:
        """
        Initialize C members of StereoHandle.
        :param cb: Callback which is passed each pair of frames
                    produced by the stereo sensors.
        """
        self.cb = cb

        # Open UVC Context
        uvc_check(uvc.uvc_init(&self.uvc_context, NULL))

        # Open stereo device handle
        optic_open_handle(
            self.uvc_context,
            &self.handle,
            optic_callback,
            &self.received_frames
        )

    def __dealloc__(self):
        optic_close_handle(self.handle)


cdef inline int uvc_check(uvc.uvc_error err) except -1:
    """
    Checks passed uvc_error value, and if it is an error, raises a
    Python OpticDeviceException with the appropriate message.
    :return 0
    :raises OpticDeviceException if uvc_error is not UVC_SUCCESS.
    """
    if err:
        raise OpticDeviceException(uvc.uvc_strerror(err))


cdef int optic_check(optic_status status) except -1:
    """
    Checks returned optic_status value, and if it is an error, raises a
    Python OpticDeviceException with the appropriate message.
    :return 0
    :raises OpticDeviceException if status is not OK.
    """
    if status:
        raise OpticDeviceException({
            kOpticDeviceNotFound: "Device not found.",
            kOpticPermissionError: "Lacking permission to access device.",
            kOpticDeviceOpenError: "Unable to open device.",
            kOpticStreamControlError: "Unable to get stream control.",
            kOpticStreamStartError: "Unable to begin device stream.",
            kOpticHandleStateError: "Handle in invalid state.",
            kOpticAllocFailure: "Unable to allocate memory",
        }[status])
    return 0
