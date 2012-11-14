/*
** Copyright 2008, Google Inc.
** Copyright (c) 2009, Code Aurora Forum. All rights reserved.
** Copyright 2010, 2011, Sony Ericsson Mobile Communciations AB
**
** Licensed under the Apache License, Version 2.0 (the "License");
** you may not use this file except in compliance with the License.
** You may obtain a copy of the License at
**
**     http://www.apache.org/licenses/LICENSE-2.0
**
** Unless required by applicable law or agreed to in writing, software
** distributed under the License is distributed on an "AS IS" BASIS,
** WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
** See the License for the specific language governing permissions and
** limitations under the License.
*/

#ifndef ANDROID_HARDWARE_QUALCOMM_CAMERA_HARDWARE_H
#define ANDROID_HARDWARE_QUALCOMM_CAMERA_HARDWARE_H

#include <CameraHardwareInterface.h>
#include <binder/MemoryBase.h>
#include <binder/MemoryHeapBase.h>
#include <stdint.h>
#include <Overlay.h>

#define CAMERA_QVGASIZE_WIDTH                 320
#define CAMERA_QVGASIZE_HEIGHT                240
#define CAMERA_WQVGASIZE_WIDTH                432
#define CAMERA_WQVGASIZE_HEIGHT               240
#define CAMERA_VGASIZE_WIDTH                  640
#define CAMERA_VGASIZE_HEIGHT                 480
#define CAMERA_WVGASIZE_WIDTH                 800
#define CAMERA_WVGASIZE_HEIGHT                480
#define CAMERA_FWVGASIZE_WIDTH                854
#define CAMERA_FWVGASIZE_HEIGHT               480
#define CAMERA_OHDFWVGASIZE_WIDTH             848
#define CAMERA_OHDFWVGASIZE_HEIGHT            480
#define CAMERA_VIDEO_FWVGASIZE_WIDTH          864
#define CAMERA_VIDEO_FWVGASIZE_HEIGHT         480
#define CAMERA_HD720PSIZE_WIDTH              1280
#define CAMERA_HD720PSIZE_HEIGHT              720
#define CAMERA_2MSIZE_WIDTH                   1632
#define CAMERA_2MSIZE_HEIGHT                  1224
#define CAMERA_FULLHDSIZE_WIDTH               1920
#define CAMERA_FULLHDSIZE_HEIGHT              1080
#define CAMERA_3MSIZE_WIDTH                   2048
#define CAMERA_3MSIZE_HEIGHT                  1536
#define CAMERA_4MWIDESIZE_WIDTH               2592
#define CAMERA_4MWIDESIZE_HEIGHT              1458
#define CAMERA_5MSIZE_WIDTH                   2592
#define CAMERA_5MSIZE_HEIGHT                  1944
#define CAMERA_6MSIZE_WIDTH                   3264
#define CAMERA_6MSIZE_HEIGHT                  1836
#define CAMERA_8MSIZE_WIDTH                   3264
#define CAMERA_8MSIZE_HEIGHT                  2448


#define COMPENSATION_M9 "-27"
#define COMPENSATION_M8 "-24"
#define COMPENSATION_M7 "-21"
#define COMPENSATION_M6 "-18"
#define COMPENSATION_M5 "-15"
#define COMPENSATION_M4 "-12"
#define COMPENSATION_M3 "-9"
#define COMPENSATION_M2 "-6"
#define COMPENSATION_M1 "-3"
#define COMPENSATION_ZERO "0"
#define COMPENSATION_P9 "27"
#define COMPENSATION_P8 "24"
#define COMPENSATION_P7 "21"
#define COMPENSATION_P6 "18"
#define COMPENSATION_P5 "15"
#define COMPENSATION_P4 "12"
#define COMPENSATION_P3 "9"
#define COMPENSATION_P2 "6"
#define COMPENSATION_P1 "3"


#define CAMERA_EXPOSURE_COMP_M9_3 0xffee0006
#define CAMERA_EXPOSURE_COMP_M8_3 0xfff00006
#define CAMERA_EXPOSURE_COMP_M7_3 0xfff20006
#define CAMERA_EXPOSURE_COMP_M6_3 0xfff40006
#define CAMERA_EXPOSURE_COMP_M5_3 0xfff60006
#define CAMERA_EXPOSURE_COMP_M4_3 0xfff80006
#define CAMERA_EXPOSURE_COMP_M3_3 0xfffa0006
#define CAMERA_EXPOSURE_COMP_M2_3 0xfffc0006
#define CAMERA_EXPOSURE_COMP_M1_3 0xfffe0006
#define CAMERA_EXPOSURE_COMP_ZERO 0x6
#define CAMERA_EXPOSURE_COMP_P1_3 0x20006
#define CAMERA_EXPOSURE_COMP_P2_3 0x40006
#define CAMERA_EXPOSURE_COMP_P3_3 0x60006
#define CAMERA_EXPOSURE_COMP_P4_3 0x80006
#define CAMERA_EXPOSURE_COMP_P5_3 0xa0006
#define CAMERA_EXPOSURE_COMP_P6_3 0xc0006
#define CAMERA_EXPOSURE_COMP_P7_3 0xe0006
#define CAMERA_EXPOSURE_COMP_P8_3 0x100006
#define CAMERA_EXPOSURE_COMP_P9_3 0x120006


//#define CAMERA_EXPOSURE_COMP_M9_3 -9 
//#define CAMERA_EXPOSURE_COMP_M8_3 -8
//#define CAMERA_EXPOSURE_COMP_M7_3 -7
//#define CAMERA_EXPOSURE_COMP_M6_3 -6
//#define CAMERA_EXPOSURE_COMP_M5_3 -5
//#define CAMERA_EXPOSURE_COMP_M4_3 -4
//#define CAMERA_EXPOSURE_COMP_M3_3 -3
//#define CAMERA_EXPOSURE_COMP_M2_3 -2
//#define CAMERA_EXPOSURE_COMP_M1_3 -1
//#define CAMERA_EXPOSURE_COMP_ZERO 6
//#define CAMERA_EXPOSURE_COMP_P1_3 1
//#define CAMERA_EXPOSURE_COMP_P2_3 2
//#define CAMERA_EXPOSURE_COMP_P3_3 3
//#define CAMERA_EXPOSURE_COMP_P4_3 4
//#define CAMERA_EXPOSURE_COMP_P5_3 5 
//#define CAMERA_EXPOSURE_COMP_P6_3 6
//#define CAMERA_EXPOSURE_COMP_P7_3 7
//#define CAMERA_EXPOSURE_COMP_P8_3 8
//#define CAMERA_EXPOSURE_COMP_P9_3 9



//Create some fake defines to please the compiler but never use since we are using an external APP and not Sony one

typedef struct camera_start_auto_focus_t{
    int camera_ae_lock;
    int camera_awb_lock;
    int camera_focus_lock;
} camera_start_auto_focus_t;

typedef struct camera_ae_awb_lock_t{
    int camera_ae_lock;
    int camera_awb_lock;
}camera_ae_awb_lock_t;


#define CAMERA_AE_LOCK 1
#define CAMERA_AE_UNLOCK 0
#define CAMERA_AWB_LOCK 1
#define CAMERA_AWB_UNLOCK 0
#define CAMERA_FOCUS_LOCK 1
#define CAMERA_FOCUS_UNLOCK 0


typedef enum {
    CAMERA_ANTIBANDING_OFF,
    CAMERA_ANTIBANDING_60HZ,
    CAMERA_ANTIBANDING_50HZ,
    CAMERA_ANTIBANDING_AUTO,
    CAMERA_MAX_ANTIBANDING,
} camera_antibanding_type;

enum camera_ops {
CAMERA_SET_PARM_DISPLAY_INFO,
CAMERA_SET_PARM_DIMENSION,
CAMERA_SET_PARM_ZOOM,
CAMERA_SET_PARM_SENSOR_POSITION,
CAMERA_SET_PARM_FOCUS_RECT,
CAMERA_SET_PARM_LUMA_ADAPTION,
CAMERA_SET_PARM_CONTRAST,
CAMERA_SET_PARM_BRIGHTNESS,
CAMERA_SET_PARM_EXPOSURE_COMPENSATION,
CAMERA_SET_PARM_SHARPNESS,
CAMERA_SET_PARM_HUE,
CAMERA_SET_PARM_SATURATION,
CAMERA_SET_PARM_EXPOSURE,
CAMERA_SET_PARM_AUTO_FOCUS,
CAMERA_SET_PARM_WB,
CAMERA_SET_PARM_EFFECT,
CAMERA_SET_PARM_FPS,
CAMERA_SET_PARM_FLASH,
CAMERA_SET_PARM_NIGHTSHOT_MODE,
CAMERA_SET_PARM_REFLECT,
CAMERA_SET_PARM_PREVIEW_MODE,
CAMERA_SET_PARM_ANTIBANDING,
CAMERA_SET_PARM_RED_EYE_REDUCTION,
CAMERA_SET_PARM_FOCUS_STEP,
CAMERA_SET_PARM_EXPOSURE_METERING,
CAMERA_SET_PARM_AUTO_EXPOSURE_MODE,
CAMERA_SET_PARM_ISO,
CAMERA_SET_PARM_BESTSHOT_MODE,
CAMERA_SET_PARM_ENCODE_ROTATION,
CAMERA_SET_PARM_PREVIEW_FPS,
CAMERA_SET_PARM_AF_MODE,
CAMERA_SET_PARM_HISTOGRAM,
CAMERA_SET_PARM_FLASH_STATE,
CAMERA_SET_PARM_FRAME_TIMESTAMP,
CAMERA_SET_PARM_STROBE_FLASH,
CAMERA_SET_PARM_FPS_LIST,
CAMERA_SET_PARM_HJR,
CAMERA_SET_PARM_ROLLOFF,
CAMERA_STOP_PREVIEW,
CAMERA_START_PREVIEW,
CAMERA_START_SNAPSHOT,
CAMERA_START_RAW_SNAPSHOT,
CAMERA_STOP_SNAPSHOT,
CAMERA_EXIT,
CAMERA_ENABLE_BSM,
CAMERA_DISABLE_BSM,
CAMERA_GET_PARM_ZOOM,
CAMERA_GET_PARM_MAXZOOM,
CAMERA_GET_PARM_AF_SHARPNESS,
CAMERA_SET_PARM_LED_MODE,
CAMERA_SET_MOTION_ISO,
CAMERA_AUTO_FOCUS_CANCEL,
CAMERA_GET_PARM_FOCUS_STEP,
CAMERA_ENABLE_AFD,
CAMERA_PREPARE_SNAPSHOT,
CAMERA_SET_FPS_MODE,
CAMERA_START_VIDEO,
CAMERA_STOP_VIDEO,
CAMERA_START_RECORDING,
CAMERA_STOP_RECORDING,
CAMERA_SET_PARM_JPEG_QUALITY,
CAMERA_SET_PARM_AF_FRAME,
CAMERA_SET_PARM_AF_RANGE,
CAMERA_GET_PARM_EXIF_PARAMS,
CAMERA_GET_ANALYSIS_DATA,
CAMERA_SET_PARM_JPEG_OUTPUT,
CAMERA_GET_PARM_JPEG_MAIN_IMG_FILE_SIZE,
CAMERA_START_AUTO_FOCUS,
CAMERA_STOP_AUTO_FOCUS,
CAMERA_SET_PARM_CAF_MODE,
CAMERA_GET_PARM_LENS_POSITION,
CAMERA_START_AUTO_ZOOM,
CAMERA_STOP_AUTO_ZOOM,
CAMERA_GET_PARM_NV_DATA,
CAMERA_GET_PARM_CAPTURE_SETTINGS,
CAMERA_SYNC_RAW_SNAPSHOT_INTERVAL,
CAMERA_GET_PARM_VERSIONS,
CAMERA_SET_AE_AWB_LOCK,
CAMERA_SET_LIMIT_MINIMUM_FPS,
CAMERA_CTRL_PARM_MAX,
};


typedef struct {
    unsigned int in1_w;
    unsigned int in1_h;
    unsigned int out1_w;
    unsigned int out1_h;
    unsigned int in2_w;
    unsigned int in2_h;
    unsigned int out2_w;
    unsigned int out2_h;
    uint8_t update_flag; 
} common_crop_t;

typedef uint8_t cam_ctrl_type;


struct camera_size_type {
    int width;
    int height;
};
struct camera_nv_data_t{
    unsigned char * nv_buf;
    uint16_t len;
    int kind;
};

#define CAMERA_INT_MAP_MAX_DIGIT 31
struct camera_int_map_type {
    int value;
};

enum {
    ASPECT_VGA,
    ASPECT_FWVGA,
    ASPECT_UWVGA,
};

typedef struct {
    int width;
    int height;
    int zoom_value;
    int smart_value;
} camerahal_maxvalue_t;

#include "esheepCameraHardware.h"

extern "C" {
#include <linux/android_pmem.h>
#include <msm_camera.h>
}

struct cam_frame_start_parms {
    unsigned int unknown;
    struct msm_frame frame;
    struct msm_frame video_frame;
};


struct str_map {
    const char *const desc;
    int val;
};

typedef enum {
    TARGET_MSM7625,
    TARGET_MSM7627,
    TARGET_QSD8250,
    TARGET_MSM7630,
    TARGET_MAX
}targetType;

struct target_map {
    const char *targetStr;
    targetType targetEnum;
};

struct board_property{
    targetType target;
    unsigned int previewSizeMask;
};

typedef enum {
    RET_NO_ERROR,
    RET_FAIL_YUV_CB,
    RET_FAIL_JPEG_CB
}retCbErrType;


typedef enum {
         CAMERA_WB_MIN_MINUS_1,
         CAMERA_WB_AUTO = 1,  /* This list must match aeecamera.h */
         CAMERA_WB_CUSTOM,
         CAMERA_WB_INCANDESCENT,
         CAMERA_WB_FLUORESCENT,
         CAMERA_WB_DAYLIGHT,
         CAMERA_WB_CLOUDY_DAYLIGHT,
         CAMERA_WB_TWILIGHT,
         CAMERA_WB_SHADE,
         CAMERA_WB_MAX_PLUS_1
} camera_wb_type;

enum {
        CAMERA_AEC_FRAME_AVERAGE,
        CAMERA_AEC_CENTER_WEIGHTED,
        CAMERA_AEC_SPOT_METERING,
};
namespace android {

#define CAMERA_EXTENSION_SIGNATURE            0xf
#define JUDGMENTBIT                           2
#define CAMERA_NORMALTHUMBNAIL_WIDTH          160
#define CAMERA_NORMALTHUMBNAIL_HEIGHT         120
#define CAMERA_CONVERGENCE_FRAME              7

#define CAMERA_DISPQVGASIZE_WIDTH             320
#define CAMERA_DISPQVGASIZE_HEIGHT            240
#define CAMERA_DISPVGASIZE_WIDTH              640
#define CAMERA_DISPVGASIZE_HEIGHT             480
#define CAMERA_DISPFWVGASIZE_WIDTH            864
#define CAMERA_DISPFWVGASIZE_HEIGHT           480
#define CAMERA_DISPVIDEO_FWVGASIZE_WIDTH      864
#define CAMERA_DISPVIDEO_FWVGASIZE_HEIGHT     480
#define CAMERA_SIGNATURE_HD720P_DISPSIZE_WIDTH  768
#define CAMERA_SIGNATURE_HD720P_DISPSIZE_HEIGHT 432
#define CAMERA_3RDPARTY_HD720P_DISPSIZE_WIDTH  1280
#define CAMERA_3RDPARTY_HD720P_DISPSIZE_HEIGHT  720

//Driver call timeout time(ms)
#define DRV_TIMEOUT_5K                        5000
#define DRV_TIMEOUT_10K                       10000
#define DRV_TIMEOUT_20K                       20000
#define DRV_TIMEOUT_100K                      100000
#define DRV_TIMEOUT_200K                      200000

#define SET_PARAM_KEY_SCENE_LENGTH            17

#define CAMERA_FWVGASIZE_DRV_WIDTH            864
#define CAMERA_FWVGASIZE_DRV_HEIGHT           480
#define CAMERA_FWVGATHUMBNAIL_WIDTH           216
#define CAMERA_FWVGATHUMBNAIL_HEIGHT          120

#define CAMERA_DEFAULT_VALUE                  "default-value"
#define CAMERA_STRING_VALUE_MAXLEN            20

#define CAMERAHAL_JPEGQUALITY_ECONOMY_MIN     1
#define CAMERAHAL_JPEGQUALITY_ECONOMY_MAX     33
#define CAMERAHAL_JPEGQUALITY_NORMAL_MIN      34
#define CAMERAHAL_JPEGQUALITY_NORMAL_MAX      66
#define CAMERAHAL_JPEGQUALITY_FINE_MIN        67
#define CAMERAHAL_JPEGQUALITY_FINE_MAX        100
#define CAMERAHAL_THUMBNAILQUALITY_FINE_MAX   100
#define CAMERAHAL_ORIENTATION_PORTRAIT        "portrait"
#define CAMERAHAL_ORIENTATION_LANDSCAPE       "landscape"
#define ORIENTATION                           "orientation"

#define SOI_MARKER_LEN                        2
#define APP1_MARKER_LEN                       2
#define APP1_SEG_LEN                          10
#define TIFF_HEADER_LEN                       8
#define IFD_NUM_LEN                           2
#define FIELD_LEN                             12
#define EXIF_DATA_OFFSET                      8

#define TALLY_VALUE_INIT                     -1
#define TALLY_VALUE_OFF                       0
#define TALLY_VALUE_FEEBLE                    1
#define TALLY_VALUE_STRONG                    255
#define FLASHLIGHT "/sys/class/leds/lv5219lg:fled/brightness"

// CLIPIT is define in camif
// Reference from IPL. (android/mm-camera/qcamera/ipl/ipl_helper.h)
#define CLIPIT(a) ( (((a)<0)||((a)>255)) \
                 ? (((a)<0)?0:255) \
                 :(a));

enum  {
    CAMERAHAL_PICSIZE_QVGA,
    CAMERAHAL_PICSIZE_VGA,
    CAMERAHAL_PICSIZE_WVGA,
    CAMERAHAL_PICSIZE_FWVGA,
    CAMERAHAL_PICSIZE_2M,
    CAMERAHAL_PICSIZE_FULLHD,
    CAMERAHAL_PICSIZE_3M,
    CAMERAHAL_PICSIZE_4MWIDE,
    CAMERAHAL_PICSIZE_5M,
    CAMERAHAL_PICSIZE_6M,
    CAMERAHAL_PICSIZE_8M,
    CAMERAHAL_PICSIZE_NOTSUPPORT
};

enum  {
    CAMERAHAL_PREVIEWSIZE_QVGA,
    CAMERAHAL_PREVIEWSIZE_VGA,
    CAMERAHAL_PREVIEWSIZE_WVGA,
    CAMERAHAL_PREVIEWSIZE_FWVGA,
    CAMERAHAL_PREVIEWSIZE_VIDEO_FWVGA,
    CAMERAHAL_PREVIEWSIZE_HD,
    CAMERAHAL_PREVIEWSIZE_NOTSUPPORT
};

enum  {
    CAMERAHAL_PREVIEW_FRAME_RATE_DEFAULT,
    CAMERAHAL_PREVIEW_FRAME_RATE_SET_FROM_SCENE,
    CAMERAHAL_PREVIEW_FRAME_RATE_SET_FROM_FRAME_RATE
};

//JPEG quality structure
typedef struct jpeg_quality_t{
    int picsize;
    uint32_t quality_ave;
    uint32_t quality_max;
    uint32_t quality_min;
} jpeg_quality_t;

/*
 * This structure defines the cropping parameters.
 * Reference from IPL(ipl_crop_struct). (android/mm-camera/qcamera/ipl/ipl_types.h)
 */
typedef struct camera_crop_struct
{
    uint32_t x;              // x pos of rectangle
    uint32_t y;              // y pos of rectangle
    uint32_t dx;             // dx of rectangle
    uint32_t dy;             // dy of rectangle
} camera_rect_type;

/*
 * This structure defines the format of an image.
 * Reference from IPL(ipl_image_struct). (android/mm-camera/qcamera/ipl/ipl_types.h)
 */

typedef struct camera_image_struct
{
    uint32_t  dx;            // Number of pixels in the x dirctn or in a row
    uint32_t  dy;            // Number of pixels in the y dirctn or in a col
    unsigned char* imgPtr; // Pointer to the image data
    uint8_t* clrPtr;         // Pointer to the Color data
} camera_image_type;

//JPEG quality table
static jpeg_quality_t jpeg_quality_sizes_economy[] = {
    {CAMERAHAL_PICSIZE_8M,     1064960, 1331200,  798720 },
    {CAMERAHAL_PICSIZE_6M,      798720,  998400,  599040 },
    {CAMERAHAL_PICSIZE_5M,      671744,  839680,  503808 },
    {CAMERAHAL_PICSIZE_4MWIDE,  503808,  626688,  376832 },
    {CAMERAHAL_PICSIZE_FULLHD,  274432,  344064,  204800 },
    {CAMERAHAL_PICSIZE_2M,      266240,  331776,  196608 },
    {CAMERAHAL_PICSIZE_VGA,      40960,   49152,   28672 },
};

static jpeg_quality_t jpeg_quality_sizes_normal[] = {
    {CAMERAHAL_PICSIZE_8M,     1662976, 1994752, 1331200 },
    {CAMERAHAL_PICSIZE_6M,     1247232, 1496064,  998400 },
    {CAMERAHAL_PICSIZE_5M,     1048576, 1257472,  839680 },
    {CAMERAHAL_PICSIZE_4MWIDE,  786432,  942080,  626688 },
    {CAMERAHAL_PICSIZE_FULLHD,  430080,  516096,  344064 },
    {CAMERAHAL_PICSIZE_2M,      413696,  495616,  331776 },
    {CAMERAHAL_PICSIZE_VGA,      61440,   73728,   49152 },
};

static jpeg_quality_t jpeg_quality_sizes_fine[] = {
    {CAMERAHAL_PICSIZE_8M,     2592768, 3194880, 1994752 },
    {CAMERAHAL_PICSIZE_6M,     1944576, 2396160, 1496064 },
    {CAMERAHAL_PICSIZE_5M,     1634304, 2015232, 1257472 },
    {CAMERAHAL_PICSIZE_4MWIDE, 1224704, 1511424,  942080 },
    {CAMERAHAL_PICSIZE_FULLHD,  671744,  827392,  516096 },
    {CAMERAHAL_PICSIZE_2M,      647168,  798720,  495616 },
    {CAMERAHAL_PICSIZE_VGA,      98304,  122880,   73728 },

};


enum{
    //SUCCESS 0 or 1
    MSM_AF_SUCCESS=0,
    MSM_AF_FAILED=1,
    MSM_AF_FORCE_STOP=2,
    MSM_AF_ERROR
};

struct msm_af_results_t{
    int msm_af_result;
    int msm_af_failurestate_info;
} msm_af_results_t;

//Autofocus result structure
struct af_result_t{
    int afstate_result;
    int af_failurestate_info;
} af_result_t;

struct capture_setting_t{
    uint32_t jpeg_main_img_buf_length;
};

struct get_versions_t{
    char project_code[5];
    char firmware[7];
    char hardware[11];
};


typedef enum{
  CAMERAHAL_STATE_IDLE,
  CAMERAHAL_STATE_INIT,
  CAMERAHAL_STATE_TAKEPICSTART,
  CAMERAHAL_STATE_PREVIEWSTOP,
  CAMERAHAL_STATE_TAKEPICDONE,
  CAMERAHAL_STATE_PREVIEWSTART,
  CAMERAHAL_STATE_AFSTART,
  CAMERAHAL_STATE_AFCANCEL,
  CAMERAHAL_STATE_AFLOCK 
} CameraHalState;


typedef struct rational_t{
    unsigned int numerator;
    unsigned int denominator;
}rational_t; 


typedef struct srational_t{
    int numerator;
    int denominator;
}srational_t; 


typedef struct camera_exif_t{
    rational_t exposure_time;
    srational_t shutter_speed_value;
    srational_t exposure_bias_value;
    short iso_speed_ratings;
    short flash;
    short subject_distance_range;
    unsigned int filler1;
    unsigned int filler2;
    unsigned int filler3;
}camera_exif_t;

typedef struct exifinfo_t{
    rational_t      exposure_time;
    srational_t     shutter_speed;
    srational_t     exposure_value;
    short           iso_speed;
    short           flash;
    short           distance_range;
}exifinfo_t;


class SemcCameraHardware : public CameraHardwareInterface {
    
public:

    virtual sp<IMemoryHeap> getPreviewHeap() const;
    virtual sp<IMemoryHeap> getRawHeap() const;

    virtual void setCallbacks(notify_callback notify_cb,
                              data_callback data_cb,
                              data_callback_timestamp data_cb_timestamp,
                              void* user);
    virtual void enableMsgType(int32_t msgType);
    virtual void disableMsgType(int32_t msgType);
    virtual bool msgTypeEnabled(int32_t msgType);

    virtual status_t dump(int fd, const Vector<String16>& args) const;
    virtual status_t startPreview();
    virtual void stopPreview();
    virtual bool previewEnabled();
    virtual status_t startRecording();
    virtual void stopRecording();
    virtual bool recordingEnabled();
    virtual void releaseRecordingFrame(const sp<IMemory>& mem);
    virtual status_t autoFocus();
    virtual status_t cancelAutoFocus();
    virtual status_t takePicture();
    virtual status_t cancelPicture();
    virtual status_t setParameters(const CameraParameters& params);
    virtual CameraParameters getParameters() const;
    virtual status_t sendCommand(int32_t command, int32_t arg1, int32_t arg2);
    virtual status_t getBufferInfo( sp<IMemory>& Frame, size_t *alignedSize);

    virtual void release();
    virtual bool useOverlay();
    virtual status_t setOverlay(const sp<Overlay> &overlay);
    static sp<CameraHardwareInterface> createInstance();
    static sp<SemcCameraHardware> getInstance();

    void receivePreviewFrame(struct msm_frame *frame);
    void receiveRecordingFrame(struct msm_frame *frame);
    void receiveJpegPicture(void);
    void jpeg_set_location();
    void receiveJpegPictureFragment(uint8_t *buf, uint32_t size);
    void notifyShutter(common_crop_t *crop, bool mPlayShutterSoundOnly);
    void receive_camframetimeout();

    void takePictureErrorCallback();

    virtual void            semcCameraExtensionService();
    virtual status_t        setParameters(CameraParameters& params, const char *key, int value);
    virtual status_t        setParameters(CameraParameters& params, const char *key, const char *value);
    virtual status_t        setPreviewSize(CameraParameters& params, int width, int height);
    virtual status_t        setPictureSize(CameraParameters& params, int width, int height);
#if SCRITCH_OFF
    virtual status_t        startAutoFocus(bool ae_lock, bool awb_lock, bool focus_lock);
    virtual status_t        stopAutoFocus();
    virtual status_t        startAutoZoom(int step_value, int skip_parm);
    virtual void            stopAutoZoom();
#endif//SCRITCH_OFF
    virtual int             getFocusPosition();
    virtual int             getStatus();
    virtual void            getPreviewSize(int *width , int *height);
#if SCRITCH_OFF
    virtual void            getSceneNvValue(void *pBuffer, int size);
    virtual void            setSceneRecognitionInfo(void *buffer, int buffer_size);
    virtual void            setCollective(void);
    virtual char*           getStringvalue(const char *key, int value);
    virtual void            forceCloseStopPreview(void);
#endif//SCRITCH_OFF

    bool converter_afresult(struct msm_af_results_t *af_result, struct af_result_t *hal_afstate_result, bool autoFocusEnabled, bool startAFEnabled);
    void convertRawcallback();
    status_t convertImage864to854(uint8_t* outBuffer, uint8_t* inBuffer);
    status_t convertImage864to848(uint8_t* outBuffer, uint8_t* inBuffer);
    bool native_get_exifinfo(int camfd);
    void receiveAutoFocus(struct msm_af_results_t *af_result);
    void receiveAutoZoom(struct msm_zoom_results_t *zoom);
    bool native_get_jpegfilesize(int camfd);
    void receiveFrameAnalysis(struct msm_analysis_data_t *analysis_data);
	status_t setJpegThumbnailSize(const CameraParameters& params);
	status_t setPreviewFpsRange(const CameraParameters& params);
private:
    SemcCameraHardware();
    virtual ~SemcCameraHardware();
    status_t startPreviewInternal();
    void stopPreviewInternal();
    friend void *auto_focus_thread(void *user);
    void runAutoFocus();
    status_t cancelAutoFocusInternal();
    bool native_set_dimension (int camfd);
    bool native_jpeg_encode (void);
    bool native_set_parm(cam_ctrl_type type, uint16_t length, void *value);
    bool native_zoom_image(int fd, int srcOffset, int dstOffset, common_crop_t *crop);

    static wp<SemcCameraHardware> singleton;

    /* These constants reflect the number of buffers that libmmcamera requires
       for preview and raw, and need to be updated when libmmcamera
       changes.
    */
    #define NUM_RECORD_BUFFERS 8
    static const int kPreviewBufferCount = NUM_PREVIEW_BUFFERS;
    static const int kRecordBufferCount = NUM_RECORD_BUFFERS;
    static const int kRawBufferCount = 1;
    static const int kJpegBufferCount = 1;

    int jpegPadding;

    CameraParameters mParameters;
    unsigned int frame_size;
    bool mCameraRunning;
    Mutex mCameraRunningLock;
    bool mPreviewInitialized;

    // This class represents a heap which maintains several contiguous
    // buffers.  The heap may be backed by pmem (when pmem_pool contains
    // the name of a /dev/pmem* file), or by ashmem (when pmem_pool == NULL).

    struct MemPool : public RefBase {
        MemPool(int buffer_size, int num_buffers,
                int frame_size,
                const char *name);

        virtual ~MemPool() = 0;

        void completeInitialization();
        bool initialized() const {
            return mHeap != NULL && mHeap->base() != MAP_FAILED;
        }

        virtual status_t dump(int fd, const Vector<String16>& args) const;

        int mBufferSize;
        int mAlignedBufferSize;
        int mNumBuffers;
        int mFrameSize;
        sp<MemoryHeapBase> mHeap;
        sp<MemoryBase> *mBuffers;

        const char *mName;
    };

    struct AshmemPool : public MemPool {
        AshmemPool(int buffer_size, int num_buffers,
                   int frame_size,
                   const char *name);
    };

    struct PmemPool : public MemPool {
        PmemPool(const char *pmem_pool,
                 int control_camera_fd, int flags, int pmem_type,
                 int buffer_size, int num_buffers,
                 int frame_size,
                 const char *name);
        virtual ~PmemPool();
        int mFd;
        int mPmemType;
        int mCameraControlFd;
        uint32_t mAlignedSize;
        struct pmem_region mSize;
    };

    sp<PmemPool> mPreviewHeap;
    sp<PmemPool> mRecordHeap;
    sp<PmemPool> mThumbnailHeap;
    sp<PmemPool> mRawHeap;
    sp<PmemPool> mDisplayHeap;
    sp<AshmemPool> mJpegHeap;
    sp<PmemPool> mRawSnapShotPmemHeap;
    sp<AshmemPool> mRawSnapshotAshmemHeap;
    sp<PmemPool> mPostViewHeap;

    sp<PmemPool> mJpegSnapShotHeap;
    sp<AshmemPool> mRawHeapSub;
    sp<AshmemPool> mRawBGRA8888Heap;
    sp<AshmemPool> mThumbnailBuffHeap;

    bool startCamera();
    bool initPreview();
    bool initRecord();
    void deinitPreview();
    bool initRaw(bool initJpegHeap);
    bool initRawSnapshot();
    void deinitRaw();
    void deinitRawSnapshot();

    bool mFrameThreadRunning;
    Mutex mFrameThreadWaitLock;
    Condition mFrameThreadWait;
    friend void *frame_thread(void *user);
    void runFrameThread(void *data);

    //720p recording video thread
    bool mVideoThreadExit;
    bool mVideoThreadRunning;
    Mutex mVideoThreadWaitLock;
    Condition mVideoThreadWait;
    friend void *video_thread(void *user);
    void runVideoThread(void *data);


    bool mShutterPending;
    Mutex mShutterLock;

    bool mSnapshotThreadRunning;
    Mutex mSnapshotThreadWaitLock;
    Condition mSnapshotThreadWait;
    friend void *snapshot_thread(void *user);
    void runSnapshotThread(void *data);
    Mutex mInSnapshotModeWaitLock;
    Condition mInSnapshotModeWait;

    void debugShowPreviewFPS() const;
    void debugShowVideoFPS() const;

    int mSnapshotFormat;
    void filterPictureSizes();
    void filterPreviewSizes();
    void storeTargetType();

    void initDefaultParameters();
    void findSensorType();
    Mutex mReleaseWaitLock;
    Condition mReleaseWait;

    status_t setPreviewSize(const CameraParameters& params);
    status_t setPictureSize(const CameraParameters& params);
    status_t setZoom(const CameraParameters& params,
                                                const char *key = NULL,
                                                const char *str_value = NULL,
                                                int int_value = -1,
                                                bool collective = false);


    status_t setJpegQuality(const CameraParameters& params, const char *key = NULL , const char *str_value  = NULL , int int_value  = -1 , bool collective = false);

    status_t setAntibanding(const CameraParameters& params, const char *key = NULL , const char *str_value  = NULL , int int_value  = -1 , bool collective = false);
    status_t setEffect(const CameraParameters& params, const char *key = NULL , const char *str_value  = NULL , int int_value  = -1 , bool collective = false);

    status_t setAutoExposure(const CameraParameters& params, const char *key = NULL , const char *str_value  = NULL , int int_value  = -1 , bool collective = false);
    status_t setWhiteBalance(const CameraParameters& params, const char *key = NULL , const char *str_value  = NULL , int int_value  = -1 , bool collective = false);

    status_t setFlash(const CameraParameters& params);
    status_t setGpsLocation(const CameraParameters& params);
    status_t setRotation(const CameraParameters& params, const char *key = NULL , const char *str_value  = NULL , int int_value  = -1);
    //status_t setZoom(const CameraParameters& params);

    status_t setFocusMode(const CameraParameters& params, const char *key = NULL , const char *str_value  = NULL , int int_value  = -1 , bool collective = false);

    status_t setBrightness(const CameraParameters& params);

    status_t setOrientation(const CameraParameters& params);

    status_t setLensshadeValue(const CameraParameters& params);
    status_t setISOValue(const CameraParameters& params);
    status_t setPictureFormat(const CameraParameters& params);
    status_t setSharpness(const CameraParameters& params);
    status_t setContrast(const CameraParameters& params);
    status_t setSaturation(const CameraParameters& params);
    void setGpsParameters();
    void storePreviewFrameForPostview();

    status_t setSceneMode(const CameraParameters& params, const char *key = NULL , const char *str_value  = NULL , int int_value  = -1 , bool collective = false);
#if 0
    status_t setSceneRecognition(const CameraParameters& params, const char *key = NULL , const char *str_value  = NULL , int int_value  = -1 , bool collective = false);

#endif//0
    status_t setExposureCompensation(const CameraParameters& params, const char *key = NULL , const char *str_value  = NULL , int int_value  = -1 , bool collective = false);
    status_t setFramerate(const CameraParameters& params, const char *key = NULL , const char *str_value  = NULL , int int_value  = -1 , bool collective = false);
    status_t setPreviewMode(const CameraParameters& params, const char *key = NULL , const char *str_value  = NULL , int int_value  = -1 , bool collective = false);
    status_t setAfMode(const CameraParameters& params, const char *key = NULL , const char *str_value  = NULL , int int_value  = -1 , bool collective = false);
    status_t setCAFMode(const CameraParameters& params, const char *key = NULL , const char *str_value  = NULL , int int_value  = -1 , bool collective = false);
#if 0
    status_t setHandJitterReduction(const CameraParameters& params, const char *key = NULL , const char *str_value  = NULL , int int_value  = -1 , bool collective = false);
    status_t setSmileMode(const CameraParameters& params, const char *key = NULL , const char *str_value  = NULL , int int_value  = -1 , bool collective = false);
    status_t setFacedetectMode(const CameraParameters& params, const char *key = NULL , const char *str_value  = NULL , int int_value  = -1 , bool collective = false);
    void setCollectiveSignature();
#endif//0
    void setCollective3rdParty();
    status_t setGpsLatitude(const CameraParameters& params, const char *key = NULL , const char *str_value  = NULL , int int_value  = -1);
    status_t setGpsLatitudeRef(const CameraParameters& params, const char *key = NULL , const char *str_value  = NULL , int int_value  = -1);
    status_t setGpsLongitude(const CameraParameters& params, const char *key = NULL , const char *str_value  = NULL , int int_value  = -1);
    status_t setGpsLongitudeRef(const CameraParameters& params, const char *key = NULL , const char *str_value  = NULL , int int_value  = -1);
    status_t setGpsAltitude(const CameraParameters& params, const char *key = NULL , const char *str_value  = NULL , int int_value  = -1);
    status_t setGpsAltitudeRef(const CameraParameters& params, const char *key = NULL , const char *str_value  = NULL , int int_value  = -1);
    status_t setGpsStatus(const CameraParameters& params, const char *key = NULL , const char *str_value  = NULL , int int_value  = -1);
    status_t setFaceDetection(const CameraParameters& params, const char *key = NULL , const char *str_value  = NULL , int int_value  = -1);
    status_t setFlashStatus(const CameraParameters& params, const char *key = NULL , const char *str_value  = NULL , int int_value  = -1);
    status_t setFlashlightBrightness(const CameraParameters& params, const char *key = NULL , const char *str_value  = NULL , int int_value  = -1);

    void semcCameraExtensionJudgment(bool startflg);
    void setInitialValue();
    int takePictureThread();
    retCbErrType startrawsnapshot();
    bool picsizeCheck(int width ,int height);
    bool previewEnabledInternal();
    bool getThumbnailInternal();
    bool convertYCbCr420lpToBgra8888(camera_image_type* input_img_ptr, camera_image_type* output_img_ptr);
    bool native_unregister_snapshot_bufs();
    void stopPreviewInternal_ForceClose();
    bool native_set_parm(cam_ctrl_type type, uint16_t length, void *value, int timeout_ms);

    void setExifParameters();
    bool receiveExifData();

    Mutex mLock;
    Mutex mCamframeTimeoutLock;
    bool camframe_timeout_flag;
    bool mReleasedRecordingFrame;

    bool receiveRawPicture(void);
    void receiveRawSnapshot(void);

    Mutex mCallbackLock;
    Mutex mOverlayLock;
	Mutex mRecordLock;
	Mutex mRecordFrameLock;
	Condition mRecordWait;
    Condition mStateWait;

    /* mJpegSize keeps track of the size of the accumulated JPEG.  We clear it
       when we are about to take a picture, so at any time it contains either
       zero, or the size of the last JPEG picture taken.
    */
    uint32_t mJpegSize;
    unsigned int        mPreviewFrameSize;
    unsigned int        mRecordFrameSize;
    int                 mRawSize;
    int                 mJpegMaxSize;

#if DLOPEN_LIBMMCAMERA
    void *libmmcamera;
    void *libmmipl;
#endif


    int mCameraControlFd;
    struct msm_camsensor_info mSensorInfo;
    cam_ctrl_dimension_t mDimension;
    bool mAutoFocusThreadRunning;
    Mutex mAutoFocusThreadLock;
    int mAutoFocusFd;

    Mutex mAfLock;

    pthread_t mFrameThread;
    pthread_t mVideoThread;
    pthread_t mSnapshotThread;

    common_crop_t mCrop;

    int mBrightness;
    struct msm_frame frames[kPreviewBufferCount];
    struct msm_frame recordframes[kRecordBufferCount];
    bool mInPreviewCallback;
    bool mUseOverlay;
    sp<Overlay>  mOverlay;

    int32_t mMsgEnabled;    // camera msg to be handled
    notify_callback mNotifyCallback;
    data_callback mDataCallback;
    data_callback_timestamp mDataCallbackTimestamp;
    void *mCallbackCookie;  // same for all callbacks
    int mDebugFps;
    int kPreviewBufferCountActual;
    int previewWidth, previewHeight;

    //notify_callback_impl mNotifyCallback_impl;
    //data_callback_impl mDataCallback_impl;

    #define SIZE_NUM (sizeof(jpeg_quality_sizes)/sizeof(jpeg_quality_t))

    Mutex                               mFrameCallbackLock;
    Mutex                               mThumnailLock;
    volatile CameraHalState             mStateCameraHal;
    Condition                           mThumnailWait;
    int                                 mThumbnailWidth;
    int                                 mThumbnailHeigth;
    int                                 mSemcCameraFlag;
    int                                 mFrameCnt;
    int                                 mCameraPicsize;
    int                                 mCameraPreviewsize;
    uint32_t                            mThumbnailsize;
    uint32_t                            mThumbnailBufferSize;
    char                                mSceneMode[CAMERA_STRING_VALUE_MAXLEN];
    char                                mWhitebalance[CAMERA_STRING_VALUE_MAXLEN];
    char                                mExposureCompensation[CAMERA_STRING_VALUE_MAXLEN];
    char                                mPreviewMode[CAMERA_STRING_VALUE_MAXLEN];
    char                                mAutoExposure[CAMERA_STRING_VALUE_MAXLEN];
    char                                mAfMode[CAMERA_STRING_VALUE_MAXLEN];
    char                                mFocusMode[CAMERA_STRING_VALUE_MAXLEN];
    char                                mHJR[CAMERA_STRING_VALUE_MAXLEN];
    char                                mSmileMode[CAMERA_STRING_VALUE_MAXLEN];
    char                                mFacedetectMode[CAMERA_STRING_VALUE_MAXLEN];
    char                                mCafMode[CAMERA_STRING_VALUE_MAXLEN];
    char                                mAntibanding[CAMERA_STRING_VALUE_MAXLEN];
    char                                mEffect[CAMERA_STRING_VALUE_MAXLEN];
    int                                 mZoom;
    exifinfo_t                          mExifInfo;
    char                                mKeyScene[SET_PARAM_KEY_SCENE_LENGTH];
    void                                *mASSBuffer;
    int                                 mASSBufferSize;
    bool                                mConfigurationFlag;

    void                                *mRecognitionFrameCallbackCookie;
    void                                *mAbstractCallbackCookie;
    bool                                mPreviewFlag;
    bool                                mAfterTakepictureFlag;
    bool                                mFocusLockValue;

    Mutex                               mCancelAutoFocus;
    Condition                           mCancelAutoFocusWait;
    bool                                mCancelAutoFocusFlg;
    Mutex                               mSetCollective3rdParty;
    Condition                           mSetCollective3rdPartyWait;
    int                                 mCurrentAspectValue;
    int                                 mFrameRate;
    int                                 mFrameRateSetFrom;
    bool                                mEncodeLocation;
    int                                 mFocusPosition;
    bool                                mTakepictureFlag;
    bool                                mTakepictureThreadJoin;
    bool                                mReceiveExifDataFlag;
    bool                                mCameraExitFlag;
    pthread_t                           mEncoderThreadInfo;
};

}; // namespace android

#endif
