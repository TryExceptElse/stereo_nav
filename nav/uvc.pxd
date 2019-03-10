from libc.stdint cimport uint8_t, uint32_t


cdef extern from 'sys/time.h':
    cdef struct timeval:
        long tv_sec
        long tv_usec


cdef extern from 'libuvc/libuvc.h':
    # libusb_context is not actually declared in libuvc, but until that
    # becomes a problem, It will be easier to leave that here.
    cdef struct libusb_context

    cdef struct uvc_context

    cdef struct uvc_device_handle

    enum uvc_frame_format:
        UVC_FRAME_FORMAT_ANY  # Any supported format.
        UVC_FRAME_FORMAT_YUYV  # YUYV/YUV2/YUV422: YUV encoding
        UVC_FRAME_FORMAT_RGB  # 24-bit RGB
        UVC_FRAME_FORMAT_MJPEG  # Motion-JPEG (or JPEG) encoded images.
        UVC_FRAME_FORMAT_COUNT  # Number of formats understood.

    cdef struct uvc_frame:
        void *                  data
        size_t                  data_bytes
        uint32_t                width
        uint32_t                height
        uvc_frame_format        frame_format
        size_t                  step
        uint32_t                sequence
        timeval                 capture_time
        uvc_device_handle *   source
        uint8_t                 library_owns_data
    
    enum uvc_error:
        UVC_SUCCESS = 0 
        UVC_ERROR_IO = -1 
        UVC_ERROR_INVALID_PARAM = -2 
        UVC_ERROR_ACCESS = -3
        UVC_ERROR_NO_DEVICE = -4 
        UVC_ERROR_NOT_FOUND = -5 
        UVC_ERROR_BUSY = -6 
        UVC_ERROR_TIMEOUT = -7
        UVC_ERROR_OVERFLOW = -8 
        UVC_ERROR_PIPE = -9 
        UVC_ERROR_INTERRUPTED = -10 
        UVC_ERROR_NO_MEM = -11
        UVC_ERROR_NOT_SUPPORTED = -12 
        UVC_ERROR_INVALID_DEVICE = -50 
        UVC_ERROR_INVALID_MODE = -51 
        UVC_ERROR_CALLBACK_EXISTS = -52
        UVC_ERROR_OTHER = -99

    uvc_error uvc_init(
        uvc_context **context,
        libusb_context *usb_context
    ) nogil

    uvc_error uvc_exit(uvc_context *context) nogil

    uvc_frame* uvc_allocate_frame(size_t size) nogil
    void uvc_free_frame(uvc_frame *frame) nogil

    uvc_error uvc_any2rgb(uvc_frame *in_frame, uvc_frame *out_frame) nogil

    void uvc_perror(uvc_error err, const char *msg) nogil

    const char* uvc_strerror(uvc_error err) nogil
