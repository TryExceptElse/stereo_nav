"""
This module handles generation of numpy arrays from the stereo device.
"""

import numpy as np


class StatusException(Exception):
    """
    Base class for exceptions storing a status or error code.
    """
    def __init__(self, code: int, *args) -> None: ...


class OpticDeviceException(StatusException):
    """
    Python Exception thrown when a UVC error occurs.
    """


class UvcError(StatusException):
    """
    Python Exception wrapping a uvc_error_t
    """


class StereoHandle:
    """
    Class handling Input from a stereo camera device.
    """
    def __init__(self, cb) -> None: ...

    def start(self) -> int: ...

    def stop(self) -> None: ...

    def read_frame(self) -> Frame: ...

    def read_pair(self) -> StereoPair: ...


class Frame:
    """
    Class wrapping stored data for a single camera's frame.
    """
    side: str
    timestamp: float
    arr: np.ndarray

    def __cinit__(self) -> None: ...

    def __dealloc__(self) -> None: ...

    def time_difference(self, other) -> float: ...


class StereoPair:
    """
    Class wrapping a pair of frames, one from each camera, at a single
    point in time.
    """
    left: Frame
    right: Frame
    time_difference: float

    def __init__(self, left: 'Frame', right: 'Frame') -> None: ...
