#ifndef OPTIC_INTF_H_
#define OPTIC_INTF_H_

#include <stdbool.h>
#include <stdint.h>

#include <libuvc/libuvc.h>


// --------------------------------------------------------------------
// Types

typedef enum optic_status {
    kOpticOk = 0,
    kOpticDeviceNotFound, 
    kOpticPermissionError,
    kOpticDeviceOpenError,
    kOpticStreamControlError,
    kOpticStreamStartError,
    kOpticHandleStateError,
    kOpticAllocFailure
} optic_status;

typedef enum optic_side {kOpticLeft, kOpticRight} optic_side;

typedef struct optic_stereo_handle optic_stereo_handle_t;

typedef struct optic_cb_data {
    enum optic_side side;
    uvc_frame_t *frame;
    void *cb_data;
} optic_cb_data_t;

// --------------------------------------------------------------------

extern const uint16_t kOpticWidth;  // Stream width
extern const uint16_t kOpticHeight;  // Stream height
extern const uint16_t kOpticFps;

// --------------------------------------------------------------------
// Functions

optic_status 
optic_open_handle               (uvc_context_t *ctx,
                                 optic_stereo_handle_t **handle,
                                 void (*cb)(optic_cb_data_t data),
                                 void *cb_data);

void 
optic_close_handle              (optic_stereo_handle_t *handle);

optic_status 
optic_handle_start_streaming    (optic_stereo_handle_t *handle);

bool
optic_handle_stop_streaming     (optic_stereo_handle_t *handle);

void 
optic_sprint_status             (char *buf, 
                                 uint16_t n, 
                                 enum optic_status status);

// --------------------------------------------------------------------

#endif  // OPTIC_INTF_H_
