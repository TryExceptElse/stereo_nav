#cython: language_level=3
"""
This module handles generation of numpy arrays from the stereo device.
"""

cimport cython
from libc.stdlib cimport malloc, free
from libc.stdint cimport uint8_t, uint16_t, uint32_t
import numpy as np
cimport numpy as np

from nav.pipe cimport *  # pipe types + functions are pipe_ prefixed.
from nav.uvc cimport *  # uvc types + functions are uvc_ prefixed.


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
        uvc_frame *frame
        void *cb_data

    # Extern vars

    const uint16_t kOpticWidth  # Stream width
    const uint16_t kOpticHeight  # Stream height
    const uint32_t kOpticFrameSize  # Frame size in bytes.
    const uint16_t kOpticFps

    # Optic Functions

    optic_status optic_open_handle(
        uvc_context *ctx,
        optic_stereo_handle_t **handle,
        void (*cb)(optic_cb_data_t data),
        void *cb_data
    ) nogil

    void optic_close_handle(optic_stereo_handle_t *handle) nogil

    optic_status optic_handle_start_streaming(
        optic_stereo_handle_t *handle
    ) nogil

    bint optic_handle_stop_streaming(optic_stereo_handle_t *handle) nogil

    void optic_sprint_status(
        char *buf,
        uint16_t n,
        optic_status status
    ) nogil


#######################################################################


cdef double MAX_SYNC_DIFF = 1 / 60  # Half of a frame duration.
cdef int PIPE_SIZE = 30


class StatusException(Exception):
    """
    Base class for exceptions storing a status or error code.
    """
    def __init__(self, code: int, *args) -> None:
        super().__init__(*args)
        self.code = code


class OpticDeviceException(StatusException):
    """
    Python Exception thrown when a UVC error occurs.
    """


class UvcError(StatusException):
    """
    Python Exception wrapping a uvc_error_t
    """


cdef struct internal_cb_data_t:
    pipe_producer_t *left_producer
    pipe_producer_t *right_producer


cdef struct frame_data_t:
    uvc_frame *frame
    optic_side side
    uvc_error err


cdef void optic_callback(optic_cb_data_t data) nogil:
    """
    This function is called by the optic interface when a frame is
    received from either sensor.

    This function is intended to be run in the uvc thread, and so
    simply translates the received frame to our desired format, and
    pushes it into the frame pipe.

    Since this function is run as part of a callback from c, exceptions
    are not thrown in this function. Instead, the err field of the
    frame_data struct may be set, indicating that something unexpected
    has occurred.

    In order to allow execution across multiple threads, this function
    does not use the gil.

    :param data: optic_cb_data_t
    :return None
    """
    cdef internal_cb_data_t *internal_cb_data = \
        <internal_cb_data_t *>data.cb_data
    cdef frame_data_t *frame_data = \
        <frame_data_t *>malloc(sizeof(frame_data_t))

    frame_data.frame = NULL
    frame_data.frame = uvc_allocate_frame(kOpticFrameSize)
    frame_data.side = data.side

    # Translate data to new frame
    frame_data.err = uvc_any2rgb(data.frame, frame_data.frame)
    if frame_data.err:
        uvc_free_frame(frame_data.frame)
        frame_data.frame = NULL

    # Push frame data into pipe.
    if data.side == kOpticLeft:
        pipe_push(internal_cb_data.left_producer, frame_data, 1)
    elif data.side == kOpticRight:
        pipe_push(internal_cb_data.right_producer, frame_data, 1)


@cython.final
cdef class StereoHandle:
    """
    Class handling Input from a stereo camera device.
    """

    cdef:
        optic_stereo_handle_t   *handle
        optic_cb_data_t         cb_data
        uvc_context             *uvc_context
        object                  cb
        pipe_t                  *frame_pipe
        pipe_consumer_t         *frame_consumer
        internal_cb_data_t      internal_cb_data
        public bint             streaming

    # If someday multiple stereo handles need to be controlled,
    # uvc_context should be added as argument to __init__.


    def __cinit__(self, cb) -> None:
        """
        Initialize C members of StereoHandle.
        :param cb: Callback which is passed each pair of frames
                    produced by the stereo sensors.
        """
        self.cb = cb
        self.streaming = False

        # Open UVC Context
        uvc_check(uvc_init(&self.uvc_context, NULL))

        # Open stereo device handle
        optic_check(optic_open_handle(
            self.uvc_context,
            &self.handle,
            optic_callback,
            &self.internal_cb_data
        ))

        # Set up frame pipe
        self.frame_pipe = pipe_new(sizeof(frame_data_t*), PIPE_SIZE)
        self.internal_cb_data.left_producer = \
            pipe_producer_new(self.frame_pipe)
        self.internal_cb_data.right_producer = \
            pipe_producer_new(self.frame_pipe)
        self.frame_consumer = pipe_consumer_new(self.frame_pipe)

    def __dealloc__(self):
        if self.streaming:
            self.stop()
        optic_close_handle(self.handle)
        if not self.frame_pipe:
            return
        pipe_free(self.frame_pipe)
        pipe_producer_free(self.internal_cb_data.left_producer)
        pipe_producer_free(self.internal_cb_data.right_producer)
        pipe_consumer_free(self.frame_consumer)

    cpdef int start(self) except -1:
        """
        Begin streaming from device.
        """
        optic_check(optic_handle_start_streaming(self.handle))
        self.streaming = True

    cpdef void stop(self):
        optic_handle_stop_streaming(self.handle)
        self.streaming = False

    cpdef Frame read_frame(self):
        """
        Read a single frame from the pipe.

        This method will block until a Frame is available.

        :return Frame
        """
        # Read a single frame data pointer from the pipe.
        cdef frame_data_t *frame_data
        cdef size_t read = pipe_pop(self.frame_consumer, &frame_data, 1)

        if read == 0:
            raise ValueError('Failed to read frame from pipe.')

        # Create wrapper object
        cdef Frame frame = Frame()
        frame.set_data(frame_data)

        return frame

    cpdef StereoPair read_pair(self):
        """
        Read a synchronized stereo frame pair from the pipe.

        This method will block until a StereoPair is available.

        :return StereoPair
        """
        cdef Frame left = None, right = None
        while (
                not (left and right) or
                abs(left.time_difference(right)) > MAX_SYNC_DIFF
        ):
            frame: Frame = self.read_frame()
            if frame.side == 'left':
                left = frame
            elif frame.side == 'right':
                right = frame
            else:
                assert False
        return StereoPair(left, right)


@cython.final
cdef class Frame:
    """
    Class wrapping stored data for a single camera's frame.
    """
    cdef frame_data_t *frame_data

    def __cinit__(self) -> None:
        self.frame_data = NULL

    def __dealloc__(self) -> None:
        if self.frame_data != NULL:
            free(self.frame_data)

    cdef void set_data(self, frame_data_t *data):
        self.frame_data = data

    cpdef double time_difference(self, Frame other):
        return (self.timestamp - other.timestamp) / 1e6

    @property
    def arr(self) -> np.ndarray:
        if self.frame_data == NULL:
            raise ValueError('Frame has not had any data set.')
        cdef uint8_t *arr_data = <uint8_t *>self.frame_data.frame.data
        cdef np.ndarray[np.uint8_t, ndim=1] np_arr = \
            np.PyArray_SimpleNewFromData(
                1, [kOpticFrameSize], np.NPY_UINT8, arr_data)
        return np_arr

    @property
    def timestamp(self) -> float:
        if self.frame_data == NULL:
            raise ValueError('Frame has not had any data set.')
        return self.frame_data.frame.capture_time.tv_usec

    @property
    def side(self) -> str:
        if self.frame_data == NULL:
            raise ValueError('Frame has not had any data set.')
        if self.frame_data.side == kOpticLeft:
            return 'left'
        elif self.frame_data.side == kOpticRight:
            return 'right'
        raise ValueError(f'Unexpected value: {self.frame_data.side}')


@cython.final
cdef class StereoPair:
    """
    Class wrapping a pair of frames, one from each camera, at a single
    point in time.
    """
    cdef public Frame left, right

    def __init__(self, left: 'Frame', right: 'Frame') -> None:
        self.left = left
        self.right = right

    @property
    def time_difference(self) -> float:
        """
        Gets the time difference between the left and right frames of
        the StereoPair
        :return float
        """
        return self.left.time_difference(self.right)


cdef inline int uvc_check(uvc_error err) except -1:
    """
    Checks passed uvc_error value, and if it is an error, raises a
    Python OpticDeviceException with the appropriate message.
    :return 0
    :raises OpticDeviceException if uvc_error is not UVC_SUCCESS.
    """
    if err:
        raise UvcError(err, uvc_strerror(err))
    return 0


cdef int optic_check(optic_status status) except -1:
    """
    Checks returned optic_status value, and if it is an error, raises a
    Python OpticDeviceException with the appropriate message.
    :return 0
    :raises OpticDeviceException if status is not OK.
    """
    if status:
        raise OpticDeviceException(status, {
            kOpticDeviceNotFound: "Device not found.",
            kOpticPermissionError: "Lacking permission to access device.",
            kOpticDeviceOpenError: "Unable to open device.",
            kOpticStreamControlError: "Unable to get stream control.",
            kOpticStreamStartError: "Unable to begin device stream.",
            kOpticHandleStateError: "Handle in invalid state.",
            kOpticAllocFailure: "Unable to allocate memory",
        }[status])
    return 0
