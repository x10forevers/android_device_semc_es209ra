/*
 * Copyright (C) 2012 Sony Mobile Communications AB.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License
 *
 */
 /**
* @file esheepCameraHardware.h
*
* esheepCameraHardware API for Camera Process.
*
*/
#ifndef ANDROID_HARDWARE_ESHEEP_CAMERA_HARDWARE_H
#define ANDROID_HARDWARE_ESHEEP_CAMERA_HARDWARE_H

#include <CameraHardwareInterface.h>


typedef enum {
    AF_MODE_NORMAL,
    AF_MODE_MACRO,
    AF_MODE_AUTO,
} isp3a_af_mode_t;

#define EXIFTAGID_GPS_LATITUDE 0x20002
#define EXIFTAGID_GPS_LATITUDE_REF 0x10001
#define EXIFTAGID_GPS_LONGITUDE 0x40004
#define EXIFTAGID_GPS_LONGITUDE_REF 0x30003
#define EXIFTAGID_GPS_ALTITUDE 0x60006
#define EXIFTAGID_GPS_ALTITUDE_REF 0x50005
#define EXIFTAGID_EXIF_CAMERA_MAKER 0x21010F
#define EXIFTAGID_EXIF_CAMERA_MODEL 0x220110
#define EXIFTAGID_EXIF_DATE_TIME_ORIGINAL 0x3A9003
#define EXIFTAGID_EXIF_DATE_TIME 0x3B9004

typedef enum {
    CAMERA_RSP_CB_SUCCESS,
    CAMERA_EXIT_CB_DONE,
    CAMERA_EXIT_CB_FAILED,
    CAMERA_EXIT_CB_DSP_IDLE,
    CAMERA_EXIT_CB_DSP_ABORT,
    CAMERA_EXIT_CB_ABORT,
    CAMERA_EXIT_CB_ERROR,
    CAMERA_EVT_CB_FRAME,
    CAMERA_EVT_CB_PICTURE,
    CAMERA_STATUS_CB,
    CAMERA_EXIT_CB_FILE_SIZE_EXCEEDED,
    CAMERA_EXIT_CB_BUFFER,
    CAMERA_EVT_CB_SNAPSHOT_DONE,
    CAMERA_CB_MAX,
} camera_cb_type;

#define CAM_CTRL_SUCCESS 1
#define CEILING16(x) (x&0xfffffff0)
#define PAD_TO_WORD(x) ((x&1) ? x+1 : x)
#define JPEG_EVENT_DONE 0


struct fifo_queue {
        int num_of_frames;
        int front;
        struct fifo_node *node;
        pthread_mutex_t mut;
        pthread_cond_t wait;
};

struct fifo_node {
        struct msm_frame *f;
        struct fifo_node *next;
};

void enqueue(struct fifo_queue *queue, struct fifo_node *node) {
        struct fifo_node *cur_node=queue->node;
        int i;
        LOGE("enqueue:%p(%d)\n", node, queue->num_of_frames);
        node->next=NULL;
        if(queue->num_of_frames==0) {
                queue->num_of_frames++;
                queue->front=!!queue->num_of_frames;
                queue->node=node;
                return;
        }
        queue->num_of_frames++;
        queue->front=!!queue->num_of_frames;
        for(i=0;i<(queue->num_of_frames-2);++i) {
                cur_node=cur_node->next;
                assert(!!cur_node);
        }
        cur_node->next=node;
}

struct fifo_node *dequeue(struct fifo_queue *queue) {
        if(queue->num_of_frames==0)
                return NULL;
        struct fifo_node *node=queue->node;
        LOGE("dequeue:%p(%d)\n", node, queue->num_of_frames);
        queue->num_of_frames--;
        queue->front=!!queue->num_of_frames;
        queue->node=queue->node->next;
        return node;
}


 typedef struct {
         uint16_t video_width;
         uint16_t video_height;
         uint16_t picture_width;
         uint16_t picture_height;
         uint16_t display_width;
         uint16_t display_height;
         uint16_t orig_picture_dx;
         uint16_t orig_picture_dy;
         uint16_t ui_thumbnail_height;
         uint16_t ui_thumbnail_width;
         uint16_t thumbnail_height;
         uint16_t thumbnail_width;
         uint16_t raw_picture_height;
         uint16_t raw_picture_width;
         uint32_t filler;
         uint16_t postview_width;
         uint16_t postview_height;
 } cam_ctrl_dimension_t;

typedef struct {
        uint32_t timestamp;  /* seconds since 1/6/1980          */
        double   latitude;   /* degrees, WGS ellipsoid */
        double   longitude;  /* degrees                */
        int16_t  altitude;   /* meters                          */
} camera_position_type;
typedef uint8_t jpeg_event_t;

typedef unsigned int exif_tag_id_t;

#define EXIF_RATIONAL 5
#define EXIF_ASCII 2
#define EXIF_BYTE 1

typedef struct {
        int val;
        int otherval;
} rat_t;


typedef union {
        char * _ascii; /* At byte 16 relative to exif_tag_entry_t */
        rat_t * _rats;
        rat_t  _rat;
        uint8_t _byte;
} exif_tag_data_t;

/* The entire exif_tag_entry_t struct must be 24 bytes in length */
typedef unsigned int exif_tag_type_t;
typedef struct {
        exif_tag_type_t type;
        uint32_t copy;
        uint32_t count;
        exif_tag_data_t data;
} exif_tag_entry_t;

typedef struct {
        exif_tag_id_t tag_id;
        exif_tag_entry_t tag_entry;
} exif_tags_info_t;


namespace android {
static const camera_size_type preview_sizes[] = {
    { 320, 240 }, // QVGA
    { 640, 480 }, //VGA
    { 800, 480 }, //WVGA
#if 0//SCRITCH_OFF
    { 848, 480 }, //QR Corder
    { 854, 480 }, //Snapshot FWVGA
    { 864, 480 }, //Video FWVGA
    { 1280, 720}, // 720P, reserved
#endif//SCRITCH_OFF
};

static const camera_size_type picture_sizes[] = {
    { 640,       480  },     //VGA
    { 1632,      1224 },     //2M
    { 1920,      1080 },     //FULL-HD
    { 3264,      1836 },     //6M-wide
    { 3264,      2448 },     //8M
};

static camerahal_maxvalue_t maxzoom_value[] = {
    {CAMERA_8MSIZE_WIDTH,      CAMERA_8MSIZE_HEIGHT,       76, 1  },
    {CAMERA_6MSIZE_WIDTH,      CAMERA_6MSIZE_HEIGHT,       76, 1  },
    {CAMERA_5MSIZE_WIDTH,      CAMERA_5MSIZE_HEIGHT,       76, 17 },
    {CAMERA_FULLHDSIZE_WIDTH,  CAMERA_FULLHDSIZE_HEIGHT,   76, 33 },
    {CAMERA_2MSIZE_WIDTH,      CAMERA_2MSIZE_HEIGHT,       76, 41 },
    {CAMERA_VGASIZE_WIDTH,     CAMERA_VGASIZE_HEIGHT,      76, 65 },
    {CAMERA_QVGASIZE_WIDTH,    CAMERA_QVGASIZE_HEIGHT,     76, 71 },
};

#define PREVIEW_SIZE_COUNT (sizeof(preview_sizes)/sizeof(camera_size_type))
#define PICTURE_SIZE_COUNT_SEMC (sizeof(picture_sizes)/sizeof(camera_size_type))
#define MAX_ZOOM_TABLE_COUNT (sizeof(maxzoom_value)/sizeof(camerahal_maxvalue_t))

static int PICTURE_SIZE_COUNT = sizeof(picture_sizes)/sizeof(camera_size_type);
static const camera_size_type * picture_sizes_ptr;

}; // namespace android
#endif
