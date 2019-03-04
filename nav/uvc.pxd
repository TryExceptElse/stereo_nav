

cdef extern from 'libuvc/libuvc.h':
    # libusb_context is not actually declared in libuvc, but until that
    # becomes a problem, It will be easier to leave that here.
    cdef struct libusb_context_t

    cdef struct uvc_context_t

    cdef struct uvc_frame_t
    
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

    uvc_error uvc_init(uvc_context_t **context, libusb_context_t *usb_context)

    uvc_error uvc_exit(uvc_context_t *context)

    void uvc_perror(uvc_error err, const char *msg)

    const char * uvc_strerror(uvc_error err)
