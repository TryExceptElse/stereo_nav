#include "optic_intf.h"

// https://ken.tossell.net/libuvc/doc/group__streaming.html

#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include <libuvc/libuvc.h>
#ifdef OPTIC_MAIN
  #include <opencv/highgui.h>
#endif  // OPTIC_MAIN

// The following are c dependencies of this file:
// install opencv (libopencv-dev)
// install libuvc (Present on ubuntu 18.04 and later)

// When compiling via command line, the following can be used:
// g++ -g -O0 -Wall -Wextra -D OPTIC_MAIN -o sample optic_intf.c 
// -lopencv_highgui -lopencv_core -luvc


static const uint32_t kLeftId = 39169;  // Product id of left sensor.
static const uint32_t kRightId = 39170;  // Product id of right sensor.
const uint16_t kOpticWidth = 640;  // Stream width
const uint16_t kOpticHeight = 480;  // Stream height
const uint32_t kOpticFrameSize = kOpticWidth * kOpticHeight * 3;
const uint16_t kOpticFps = 30;

// --------------------------------------------------------------------

typedef struct optic_cb_wrapper {
    void (*cb)(optic_cb_data_t data);
    optic_cb_data_t cb_data;
} optic_cb_wrapper_t;

struct optic_stereo_handle_t {
    uvc_device_t *left;
    uvc_device_t *right;
    uvc_device_handle_t *left_handle;
    uvc_device_handle_t *right_handle;
    uvc_stream_ctrl_t left_ctrl;
    uvc_stream_ctrl_t right_ctrl;
    bool left_is_streaming;
    bool right_is_streaming;
    optic_cb_wrapper_t left_cb_wrapper;
    optic_cb_wrapper_t right_cb_wrapper;
};

// --------------------------------------------------------------------


static void     
optic_print_uvc_devices     (uvc_context_t *ctx);

static optic_status     
optic_find_devices          (uvc_context_t *ctx, 
                             uvc_device_t **left,
                             uvc_device_t **right);

static optic_status
optic_set_up_sensor         (uvc_device_t *device, 
                             uvc_device_handle_t **handle,
                             uvc_stream_ctrl_t *ctrl);

static void optic_cb        (uvc_frame_t *frame, void *data);


// --------------------------------------------------------------------

#ifdef OPTIC_MAIN

/* This callback function runs once per frame. Use it to perform any
 * quick processing you need, or have it put the frame into your application's
 * input queue. If this function takes too long, you'll start losing frames. */
void cb(uvc_frame_t *frame, void *ptr) {
  uvc_frame_t *bgr;
  uvc_error_t ret;
  /* We'll convert the image from YUV/JPEG to BGR, so allocate space */
  bgr = uvc_allocate_frame(frame->width * frame->height * 3);
  if (!bgr) {
    printf("unable to allocate bgr frame!");
    return;
  }
  /* Do the BGR conversion */
  ret = uvc_any2bgr(frame, bgr);
  if (ret) {
    uvc_perror(ret, "uvc_any2bgr");
    uvc_free_frame(bgr);
    return;
  }
  IplImage *cvImg;
  cvImg = cvCreateImageHeader(
         cvSize(bgr->width, bgr->height),
         IPL_DEPTH_8U,
         3);

  cvSetData(cvImg, bgr->data, bgr->width * 3);

  cvNamedWindow("Test", CV_WINDOW_AUTOSIZE);
  cvShowImage("Test", cvImg);
  cvWaitKey(10);
  cvReleaseImageHeader(&cvImg);
  uvc_free_frame(bgr);
}

int main(int argc, char **argv) {
  uvc_context_t *ctx;
  uvc_device_t *dev;
  uvc_device_handle_t *devh;
  uvc_stream_ctrl_t ctrl;
  uvc_error_t res;
  /* Initialize a UVC service context. Libuvc will set up its own libusb
   * context. Replace NULL with a libusb_context pointer to run libuvc
   * from an existing libusb context. */
  res = uvc_init(&ctx, NULL);
  if (res < 0) {
    uvc_perror(res, "uvc_init");
    return res;
  }
  puts("UVC initialized");

  optic_print_uvc_devices(ctx);

  /* Locates the first attached UVC device, stores in dev */
  res = uvc_find_device(
      ctx, &dev,
      0, kLeftId, NULL); /* filter devices: vendor_id, product_id, "serial_num" */
  if (res < 0) {
    uvc_perror(res, "uvc_find_device"); /* no devices found */
  } else {
    puts("Device found");
    /* Try to open the device: requires exclusive access */
    res = uvc_open(dev, &devh);
    if (res < 0) {
      uvc_perror(res, "uvc_open"); /* unable to open device */
    } else {
      puts("Device opened");
      /* Print out a message containing all the information that libuvc
       * knows about the device */
      uvc_print_diag(devh, stderr);
      /* Try to negotiate a 640x480 30 fps YUYV stream profile */
      res = uvc_get_stream_ctrl_format_size(
          devh, &ctrl, /* result stored in ctrl */
          UVC_FRAME_FORMAT_YUYV, /* YUV 422, aka YUV 4:2:2. try _COMPRESSED */
          640, 480, 30 /* width, height, fps */
      );
      /* Print out the result */
      uvc_print_stream_ctrl(&ctrl, stderr);
      if (res < 0) {
        uvc_perror(res, "get_mode"); /* device doesn't provide a matching stream */
      } else {
        /* Start the video stream. The library will call user function cb:
         *   cb(frame, (void*) 12345)
         */
        res = uvc_start_streaming(devh, &ctrl, cb, (void*)12345, 0);
        if (res < 0) {
          uvc_perror(res, "start_streaming"); /* unable to start stream */
        } else {
          puts("Streaming...");
          uvc_set_ae_mode(devh, 1); /* e.g., turn on auto exposure */
          usleep(30 * 1000 * 1000); /* stream for 10 seconds */
          /* End the stream. Blocks until last callback is serviced */
          uvc_stop_streaming(devh);
          puts("Done streaming.");
        }
      }
      /* Release our handle on the device */
      uvc_close(devh);
      puts("Device closed");
    }
    /* Release the device descriptor */
    uvc_unref_device(dev);
  }
  /* Close the UVC context. This closes and cleans up any existing device handles,
   * and it closes the libusb context if one was not provided. */
  uvc_exit(ctx);
  puts("UVC exited");
  return 0;
}

#endif  // OPTIC_MAIN

// --------------------------------------------------------------------
// Public

enum optic_status optic_open_handle(
        uvc_context_t *ctx,
        optic_stereo_handle_t **handle,
        void (*cb)(optic_cb_data_t data),
        void *cb_data) {

    // Allocate optic handle.
    *handle = (optic_stereo_handle_t *) malloc(sizeof(optic_stereo_handle_t));
    if (!*handle) {
       return kOpticAllocFailure; 
    }
    
    // Set fields to initial values so that in the event of partial
    // initialization, we can clean up.
    (*handle)->left = NULL;
    (*handle)->right = NULL;
    (*handle)->left_handle = NULL;
    (*handle)->right_handle = NULL;
    
    // Set data passed in by caller.
    (*handle)->left_cb_wrapper.cb = cb;
    (*handle)->right_cb_wrapper.cb = cb;
    (*handle)->left_cb_wrapper.cb_data.cb_data = cb_data;
    (*handle)->right_cb_wrapper.cb_data.cb_data = cb_data;
    
    // Set up cb data
    (*handle)->left_cb_wrapper.cb_data.side = kOpticLeft;
    (*handle)->right_cb_wrapper.cb_data.side = kOpticRight;

    (*handle)->left_is_streaming = false;
    (*handle)->right_is_streaming = false;
    
    // Find devices.

    if (optic_find_devices(ctx, &(*handle)->left, &(*handle)->right)) {
        optic_close_handle(*handle);
        return kOpticDeviceOpenError;
    }

    // Set up handles for devices.
    optic_status status;
    status = optic_set_up_sensor(
            (*handle)->left, 
            &(*handle)->left_handle,
            &(*handle)->left_ctrl
    );
    if (status) return status;
    status = optic_set_up_sensor(
            (*handle)->right, 
            &(*handle)->right_handle,
            &(*handle)->right_ctrl
    );
    if (status) return status;
    return kOpticOk;
}

void optic_close_handle(optic_stereo_handle_t *handle) {
    // Close handles and devices.
    // This function should not assume that the handle was correctly
    // or fully initialized.
    if (handle->left_handle) {
        uvc_close(handle->left_handle);
        handle->left_handle = NULL;
    }
    if (handle->right_handle) {
        uvc_close(handle->right_handle);
        handle->right_handle = NULL;
    }
    if (handle->left) {
        uvc_unref_device(handle->left);
        handle->left = NULL;
    }
    if (handle->right) {
        uvc_unref_device(handle->right);
        handle->right = NULL;
    }
    free(handle);
}

optic_status optic_handle_start_streaming(optic_stereo_handle_t *handle) {
    // Check handle is not already streaming.
    if (handle->left_is_streaming || handle->right_is_streaming) {
        return kOpticHandleStateError;
    }
    
    uvc_error_t res;

    // Begin left stream.
    res = uvc_start_streaming(
            handle->left_handle,
            &handle->left_ctrl,
            optic_cb,
            (void *)&handle->left_cb_wrapper,
            0
    );
    if (res < 0) {
        puts("Unable to begin left stream.");
        uvc_perror(res, "uvc_start_streaming");
        return kOpticStreamStartError;
    } else {
        uvc_set_ae_mode(handle->left_handle, 1); // turn on auto exposure.
        handle->left_is_streaming = true;
    }

//    // Begin right stream.
//    res = uvc_start_streaming(
//            handle->right_handle,
//            &handle->right_ctrl,
//            optic_cb,
//            (void *)&handle->right_cb_wrapper,
//            0
//    );
//    if (res < 0) {
//        puts("Unable to begin right stream.");
//        uvc_perror(res, "uvc_start_streaming");
//        return kOpticStreamStartError;
//    } else {
//        uvc_set_ae_mode(handle->right_handle, 1); // turn on auto exposure.
//        handle->right_is_streaming = true;
//    }
    return kOpticOk;
}

bool optic_handle_stop_streaming(optic_stereo_handle_t *handle) {
    bool rv = false;
    if (handle->left_is_streaming) {
        uvc_stop_streaming(handle->left_handle);
        handle->left_is_streaming = false;
        rv = true;
    }
    if (handle->right_is_streaming) {
        uvc_stop_streaming(handle->right_handle);
        handle->right_is_streaming = false;
        rv = true;
    }
    return rv;
}

// --------------------------------------------------------------------

static optic_status optic_find_devices(
        uvc_context_t *ctx, uvc_device_t **left, uvc_device_t **right) {
    uvc_error_t res;
  
    // Display list of connected devices.
    optic_print_uvc_devices(ctx);
    // Find left + right devices.
    res = uvc_find_device(
        ctx, 
        left,
        0,  // vendor_id
        kLeftId,  // product id
        NULL  // serial num
    );
    if (res < 0) {
        puts("Unable to find left sensor.");
        uvc_perror(res, "uvc_find_device");
        return kOpticDeviceNotFound;
    }
    res = uvc_find_device(
        ctx, 
        right,
        0,  // vendor_id
        kRightId,  // product id
        NULL  // serial num
    );
    if (res < 0) {
        puts("Unable to find right sensor.");
        uvc_perror(res, "uvc_find_device");
        return kOpticDeviceNotFound;
    }
    return kOpticOk;
}

static optic_status optic_set_up_sensor(
    uvc_device_t *device, 
    uvc_device_handle_t **handle,
    uvc_stream_ctrl_t *ctrl
) {
    uvc_error_t res;
    res = uvc_open(device, handle);
    if (res < 0) {
        puts("Unable to open sensor handle");
        uvc_perror(res, "uvc_open"); /* unable to open device */
        return kOpticDeviceOpenError;
    }
    res = uvc_get_stream_ctrl_format_size(
        *handle, ctrl, /* result stored in ctrl */
        UVC_FRAME_FORMAT_YUYV, /* YUV 422, aka YUV 4:2:2. try _COMPRESSED */
        kOpticWidth, kOpticHeight, kOpticFps /* width, height, fps */
    );
    if (res < 0) {
        puts("Unable to get stream control");
        uvc_perror(res, "uvc_get_stream_ctrl");
        return kOpticStreamControlError;
    }
    return kOpticOk;
}

static void optic_cb(uvc_frame_t *frame, void *data) {
    optic_cb_wrapper_t *wrapper = (optic_cb_wrapper_t *)data;
    wrapper->cb_data.frame = frame;
    wrapper->cb(wrapper->cb_data);
}

static void optic_print_uvc_devices(uvc_context_t *ctx) {
    uvc_error_t res;
    uvc_device_t **list;
    res = uvc_get_device_list(ctx, &list);
    if (res < 0) {
        puts("Unable to get list");
        uvc_perror(res, "uvc_get_device_list");
        return;
    }

    uvc_device_descriptor_t *desc;
    for(uint16_t i = 0; list[i] != NULL; ++i) {
        res = uvc_get_device_descriptor(list[i], &desc);
        if (res < 0) {
            puts("Unable to get device descriptor");
            uvc_perror(res, "uvc_get_device_descriptor");
            return;
        }
        printf("\nDevice idVendor: %d\n", (int)desc->idVendor);
        printf("Device idProduct: %d\n", (int)desc->idProduct);
        printf("Device bcdUVC: %d\n", (int)desc->bcdUVC);
        printf("Device Serial Number: %s\n", desc->serialNumber);
        printf("Device manufacturer: %s\n", desc->manufacturer);
        printf("Device product identifier: %s\n", desc->product);
        uvc_free_device_descriptor(desc);
    }

    uvc_free_device_list(list, true);
}
