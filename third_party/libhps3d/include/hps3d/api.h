/**********************************************************************
* COPYRIGHT NOTICE - HYPERSEN TECHNOLOGY
*
* Copyright (c) 2018, Hypersen Technology, Inc.
*
* All rights reserved.
*
*======================================================================
* \file api.h
* \author Kevin
* \email Kevin_Wang@hypersen.com
* \version 1.0.0
* \date 2018年11月13日 上午11:43:31
* \license private & classified
*---------------------------------------------------------------------
* Remark: This project is still under construction.
*======================================================================
* Change History:
*---------------------------------------------------------------------
* <Date> | <Version>  | <Description>
*---------------------------------------------------------------------
* 2018年11月13日
================================================================
* Detailed Notes:
*---------------------------------------------------------------------
* <Version> -------------------------------------------------------------------
* V1.0.0 ----------------------------------------------------------

**********************************************************************/

#ifndef HPS3D_API_H_
#define HPS3D_API_H_

#ifdef __cplusplus
extern "C" /*C++*/
{
#endif

#include <stdbool.h>
#include <string.h>

#ifdef _WIN32
#include <windows.h>
#include <winsock2.h>
#else
#include <netinet/in.h>
#include <sys/socket.h>
#endif

#define DLL_API _declspec(dllexport)

typedef signed char int8_t;
typedef unsigned char uint8_t;
typedef unsigned short uint16_t;
typedef short int16_t;
typedef unsigned int uint32_t;
typedef int int32_t;
typedef float float32_t;
typedef double float64_t;

/***********************************************************************************/
/*User can modify the parameter,odify parameter values will affect the
 * occupation of the SDK memory. Refer to SDK manual(HPS3D_RM002) for detail
 * instructions. 4 Support mult-devices connection  */
#define DEV_NUM (10)       /*Device number supported*/
#define DEV_NAME_SIZE (20) /*Device name size*/
#define ROI_NUM (8)        /*Number of ROI*/
#define OBSTACLE_NUM (20)  /*Number of obstacles supported*/
#define OBSERVER_NUM (10)  /*Number of observers*/
/**********************************************************************************/
/*Parameters for internal use only, do not modify*/
/* olution */
#define RES_WIDTH (160)
#define RES_HEIGHT (60)
#define MAX_PIX_NUM (9600) /*RES_WIDTH * RES_HEIGHT*/

/*Indicates the invalid data and meaning of current measuring pixel*/
#define LOW_AMPLITUDE (65300) /*Low amplitude*/
#define SATURATION (65400)    /*Saturation*/
#define ADC_OVERFLOW (65500)  /*ADC overflow*/
#define INVALID_DATA (65530)  /*Indalid data*/
/**********************************************************************************/

/*The result returned by the function*/
typedef enum {
    RET_OK = 0x01,
    RET_ERROR = 0x02,
    RET_BUSY = 0x03,
    RET_CONNECT_FAILED,
    RET_CREAT_PTHREAD_ERR, /*Thread creation failed*/
    RET_WRITE_ERR,
    RET_READ_ERR,
    RET_PACKET_HEAD_ERR,
    RET_PACKET_ERR,
    RET_BUFF_EMPTY,   /*Buffer is empty or unavailable*/
    RET_VER_MISMATCH, /*Camera firmware version does not match SDK version*/
} RET_StatusTypeDef;

/*Device version*/
typedef struct {
    uint8_t year;
    uint8_t month;
    uint8_t day;
    uint8_t major;
    uint8_t minor;
    uint8_t rev;
} Version_t;

/*Run mode*/
typedef enum {
    MinOfRunModeType = 0,
    RUN_IDLE = 0,    /*Standby or stop measuring*/
    RUN_SINGLE_SHOT, /*Single measurement*/
    RUN_CONTINUOUS,  /*Continuous measurement*/
    NumberOfRunModeType
} RunModeTypeDef;

/*Set measurement data package type*/
typedef enum {
    DEPTH_DATA_PACKET = 0x0, /*Depth data packet  @see DepthDataTypeDef*/
    ROI_DATA_PACKET,         /*ROI data packet  @see FullRoiDataTypeDef  @see
                                SimpleRoiDataTypeDef*/
    OBSTACLE_PACKE           /*Obstacle data type  @see ObstacleDataTypedef*/
} MeasurePacketTypeDef;

/*Output data packet type*/
typedef enum {
    PACKET_FULL = 0, /*Full data packet(includes depth data)*/
    PACKET_SIMPLE    /*Simple data packet(depth data not included*/
} OutPacketTypeDef;

/*ROI threshold alarm type*/
typedef enum {
    ROI_ALARM_DISABLE = 0, /* Disable ROI GPIO alam*/
    ROI_ALARM_GPIO         /*ROI alam type is GPIO signal output*/
} ROIAlarmTypeDef;

/*Hysteresis configuration*/
typedef struct {
    uint8_t threshold_id;     /*ROI threshold id*/
    uint32_t threshold_value; /*ROI threshold value*/
    uint32_t hysteresis;      /*Hysteresis value*/
    bool enable;              /*Enable hysteresis*/
    bool positive;            /*true:Positive comparison, return True if the input value is
                                 larger than the threshold false:Reverse comparison, return
                                 False if the input value is smaller than the threshold*/
} HysteresisSingleConfTypeDef;

/*ROI reference value type*/
typedef enum {
    ROI_REF_DIST_AVR = 1,
    ROI_REF_DIST_MIN,
    ROI_REF_DIST_MAX,
    ROI_REF_SAT_COUNT,
    ROI_REF_AMPLITUDE,
    ROI_REF_VAILD_AMPLITUDE,
    ROI_REF_THRESHOLD_PIX_NUM
} ROIReferenceTypeDef;

/*ROI configured structure*/
typedef struct {
    bool enable;
    uint8_t roi_id;
    uint16_t left_top_x;
    uint16_t left_top_y;
    uint16_t right_bottom_x;
    uint16_t right_bottom_y;
    HysteresisSingleConfTypeDef hysteresis_conf[3];
    ROIReferenceTypeDef ref_type[3];    /*ROI reference value type corresponds to hysteresis_conf*/
    ROIAlarmTypeDef alarm_type[3];      /*ROI threshold alarm type corresponds to hysteresis_conf*/
    uint16_t pixel_number_threshold[3]; /*Pixel number of over threshold
                                           corresponds to hysteresis_conf*/
} ROIConfTypeDef;

/*HDR Mode*/
typedef enum {
    HDR_DISABLE = 0, /*Not recommended for use*/
    AUTO_HDR,        /*Not recommended for use*/
    SUPER_HDR,
    SIMPLE_HDR
} HDRModeTypeDef;

/*HDR Configuration*/
typedef struct {
    HDRModeTypeDef hdr_mode;
    float32_t qualtity_overexposed;         /*AUTO_HDR Overexposed amplitude, this mode is
                                               not recommended*/
    float32_t qualtity_overexposed_serious; /*AUTO_HDR Serious overexposed amplitude,
                                               this mode is not recommended*/
    float32_t qualtity_weak;                /*AUTO_HDR Weak signal amplitude, this mode is not
                                               recommended*/
    float32_t qualtity_weak_serious;        /*AUTO_HDR Serious weak signal amplitude,
                                               this mode is not recommended*/
    uint32_t simple_hdr_max_integration;    /*SIMPLE_HDR Maximum integration time us*/
    uint32_t simple_hdr_min_integration;    /*SIMPLE_HDR Minimum integration time us*/
    uint8_t super_hdr_frame_number;         /*SUPER_HDR Synthetic frame numbe*/
    uint32_t super_hdr_max_integration;     /*SUPER_HDR Maximum integration time us*/
    uint32_t hdr_disable_integration_time;  /*HDR_DISABLE Manual integration time
                                               us this mode is not recommended*/
} HDRConf;

/*Smooth filter type*/
typedef enum {
    SMOOTH_FILTER_DISABLE = 0, /*Disalbe smooth filter*/
    SMOOTH_FILTER_AVERAGE = 1, /*Average filter*/
    SMOOTH_FILTER_GAUSS        /*Gauss filter*/
} SmoothFilterTypeDef;

/*Smooth filter configuration*/
typedef struct {
    SmoothFilterTypeDef type; /*Smooth filter type*/
    uint32_t arg1;            /*Filter parameter*/
} SmoothFilterConfTypeDef;

/*uart conifg param*/
typedef enum {
    UART_BAUDRATE_9600 = 9600,
    UART_BAUDRATE_19200 = 19200,
    UART_BAUDRATE_38400 = 38400,
    UART_BAUDRATE_57600 = 57600,
    UART_BAUDRATE_115200 = 115200,
    UART_BAUDRATE_230400 = 230400,
    UART_BAUDRATE_460800 = 460800,
    UART_BAUDRATE_1000000 = 1000000,
    UART_BAUDRATE_1500000 = 1500000,
    UART_BAUDRATE_2000000 = 2000000,
} UartBaudrateTypeDef;

typedef enum {
    PARITY_NONE1,
    PARITY_ODD1,
    PARITY_EVEN1,
} UartParityTypeDef;

typedef enum {
    UART_STOP_BITS0_5,
    UART_STOP_BITS1,
    UART_STOP_BITS1_5,
    UART_STOP_BITS2,
} UartStopBitTypeDef;

typedef struct {
    UartBaudrateTypeDef baudrate;
    UartParityTypeDef parity;
    UartStopBitTypeDef stop_bits;
} UartConfTypeDef;

/*GPIO configuration definition*/
/*GPIO_OUT function*/
typedef enum {
    GPOUT_FUNC_DISABLE = 0,          /*GPIO alam disalbe*/
    GPOUT_FUNC_ROI_THRESHOLD0_ALARM, /*GPIO output threshold 0 alarm*/
    GPOUT_FUNC_ROI_THRESHOLD1_ALARM, /*GPIO output threshold 1 alarm*/
    GPOUT_FUNC_ROI_THRESHOLD2_ALARM  /*GPIO output threshold 2 alarm*/
} GPOutFunctionTypeDef;

/*GPIO_IN function*/
typedef enum {
    GPIN_FUNC_DISABLE = 0, /*GPIO function disable*/
    GPIN_FUNC_CAPTURE      /*Start measuring. Note:After the measurement is turned on,
                              it is not controlled by the command and is controlled
                              only by the IO input.*/
} GPInFunctionTypeDef;

/*GPIO Polarity*/
typedef enum { GPIO_POLARITY_LOW = 0, GPIO_POLARITY_HIGH } GPIOPolarityTypeDef;

/*GPIO Pin*/
typedef enum {
    GPIN_1 = 1,
    GPIN_2 = 2,
    GPIN_3 = 3,
    GPOUT_1 = 10,
    GPOUT_2 = 11,
    GPOUT_3 = 12,
} GPIOTypeDef;

/*GPIO output configuration*/
typedef struct {
    GPIOTypeDef gpio;
    GPIOPolarityTypeDef polarity;
    GPOutFunctionTypeDef function;
} GPIOOutConfTypeDef;

/*GPIO input configuration*/
typedef struct {
    GPIOTypeDef gpio;
    GPIOPolarityTypeDef polarity;
    GPInFunctionTypeDef function;
} GPIOInConfTypeDef;

/*Depth filter type*/
typedef enum {
    DISTANCE_FILTER_DISABLE = 0,
    DISTANCE_FILTER_SIMPLE_KALMAN /*Simple Kalman filter*/
} DistanceFilterTypeDef;

typedef struct {
    DistanceFilterTypeDef filter_type; /*@see DistanceFilterTypeDef*/
    float32_t kalman_K;                /*Scaling factor K. recommended value 0.5*/
    uint32_t kalman_threshold;         /*Noise threshold*/
    uint32_t num_check;                /*Threshold check frame number, recommended value 2*/
} DistanceFilterConfTypeDef;

/*Installation angle change parameters, for rotating coordinate system use*/
typedef struct {
    bool enable;            /*Enable installation angle*/
    uint8_t angle_vertical; /*Vertical installation angle（°）:The angle between
                               the main optical axis and the ground perpendicula*/
    uint16_t height;        /*Mounting height relative to the ground(mm)*/
} MountingAngleParamTypeDef;

/*Returned data packet type*/
typedef enum {
    NULL_PACKET = 0x00,       /*Data returns empty means no measurement data is returned*/
    SIMPLE_ROI_PACKET = 0x01, /*Simple ROI data packet(depth data not included
                                 @see SimpleRoiDataTypeDef*/
    FULL_ROI_PACKET,          /*Full ROI packet(includes depth data)@see
                                 FullRoiDataTypeDef*/
    FULL_DEPTH_PACKET,        /*Full depth data packet(includes depth data)@see
                                 DepthDataTypeDef*/
    SIMPLE_DEPTH_PACKET,      /*Simple depth data packet(depth data not included)@see
                                 DepthDataTypeDef*/
    OBSTACLE_PACKET,          /*Obstacle data packet @see ObstacleDataTypedef*/
    SYSTEM_ERROR              /*System error*/
} RetPacketTypedef;

/*ROI simple data packet*/
typedef struct {
    uint8_t group_id;                     /*group ID*/
    uint8_t id;                           /*ROI ID*/
    uint16_t amplitude;                   /*average amplitude*/
    uint16_t valid_amplitude;             /*average valid amplitude*/
    uint16_t distance_average;            /*average distance*/
    uint16_t distance_max;                /*maximum distance*/
    uint16_t distance_min;                /*minimum distance*/
    uint16_t dist_max_x;                  /*Unavailable*/
    uint16_t dist_max_y;                  /*Unavailable*/
    uint16_t dist_min_x;                  /*Unavailable*/
    uint16_t dist_min_y;                  /*Unavailable*/
    uint16_t saturation_count;            /*Number of saturated pixels*/
    uint8_t threshold_state;              /*Alarm indicator, indicates if the current measured
                                             value exceeds the threshold :bit0:zone0,
                                             bit1:zone1, bit2:zone2*/
    uint16_t out_of_threshold_pix_num[3]; /*[0]:pixel number exceed
                                             thresold0,[1]:...,[2]:...*/
    uint16_t frame_cnt;                   /*Frame counter*/
} SimpleRoiDataTypeDef;

/*Full ROI data packet*/
typedef struct {
    uint8_t roi_num;           /*ROI total number*/
    uint8_t group_id;          /*group ID*/
    uint8_t id;                /*ROI ID*/
    uint16_t left_top_x;       /*Upper left corner x coordinate*/
    uint16_t left_top_y;       /*Upper left corner y coordinate*/
    uint16_t right_bottom_x;   /*Right lower corner x coordinate*/
    uint16_t right_bottom_y;   /*Right lower corner y coordinate*/
    uint32_t pixel_number;     /*ROI pixel number*/
    uint16_t amplitude;        /*Average amplitude*/
    uint16_t valid_amplitude;  /*Average valid amplitute*/
    uint16_t distance_average; /*Average distance*/
    uint16_t distance_max;     /*Unavailable*/
    uint16_t distance_min;     /*Minimum distance*/
    uint16_t saturation_count; /*Number of saturated pixels*/
    uint16_t threshold_state;  /*if the current measured value exceeds the
                                  threshold:bit0:zone0, bit1:zone1, bit2:zone2*/
    uint16_t dist_max_x;       /*Unavailable*/
    uint16_t dist_max_y;       /*Unavailable*/
    uint16_t dist_min_x;       /*Unavailable*/
    uint16_t dist_min_y;       /*Unavailable*/
    uint32_t frame_cnt;        /*Frame counter*/
    uint16_t* distance;        /*Depth data, store in order*/
} FullRoiDataTypeDef;

/*Depth data*/
typedef struct {
    uint16_t distance_average;        /*Average distance value of the whole field of view*/
    uint16_t amplitude_average;       /*Valid amplitude, invalid pixels not included*/
    uint16_t amplitude_average_whole; /*Average amplitude value of the whole field
                                         of view*/
    uint16_t amplitude_low_count;     /*Number of low amplitute pixel*/
    uint16_t saturation_count;        /*Number of saturated pixels*/
    uint16_t distance_max;            /*Unavailable*/
    uint16_t distance_min;            /*Minimum estimated distance*/
    int16_t temperature;              /*Camera current reference temperature*/
    uint16_t frame_cnt;               /*Frame counter,can be used for frame loss detection*/
    uint16_t interference_num;        /*Unavailable*/
    uint16_t* distance;               /*Depth data, store in order, not available when the
                                         output data type is @see PACKET_SIMPLE*/
} DepthDataTypeDef;

typedef struct {
    float32_t x;
    float32_t y;
    float32_t z;
} PerPointCloudDataTypeDef;

/*Ordered point cloud data*/
typedef struct {
    PerPointCloudDataTypeDef* point_data;
    uint16_t width;
    uint16_t height;
    uint32_t points;
} PointCloudDataTypeDef;

/*Obstacle configuration parameters*/
typedef struct {
    bool enable;
    uint16_t frame_head;     /*Data frame, frame header feature byte, such as 0XEB81*/
    uint8_t number;          /*The number of obstacles that need to be extracted, for
                                example 3*/
    uint16_t vaild_min_dist; /*The minimum distance of the effective range in mm,
                                for example 60*/
    uint16_t vaild_max_dist; /*The maximum distance of the effective range in mm,
                                for example 3500*/
    uint16_t invaild_value;  /*Fixed parameter value of invalid area in mm eg 5000*/
    uint16_t frame_size;     /*Save the current buffer valid bytes*/
} ObstacleConfigTypedef;

/*Obstacle data*/
typedef struct {
    uint8_t ObstacleNumber; /*Obstacle total number*/
    uint8_t Id;             /*Obstacle ID*/
    uint32_t FrameCount;    /*Framce count value*/
    uint16_t PixelNumber;
    uint16_t DistanceAverage;
    PerPointCloudDataTypeDef LeftPoint;
    PerPointCloudDataTypeDef RightPoint;
    PerPointCloudDataTypeDef UpperPoint;
    PerPointCloudDataTypeDef UnderPoint;
    PerPointCloudDataTypeDef MinPoint;
    PerPointCloudDataTypeDef* PixelBuffer; /*Save all pixel information for obstacles*/
} ObstacleDataTypedef;

/*Obstacle avoidance parameter structure*/
typedef struct {
    bool avoidance_enable;
    uint32_t avoidance_width;
    uint32_t avoidance_height;
    uint32_t avoidance_minDistance;
    uint32_t avoidance_mountHeight; /*Mount height*/
    uint32_t avoidance_pixelNumber; /*Pixcel threshold number*/
    bool avoidance_alarmStatues;    /*Alarm status*/
    GPIOOutConfTypeDef GpioOutConf; /*GPIO function*/
} AvoidanceTypeDef;

/*Data return*/
typedef struct {
    SimpleRoiDataTypeDef* simple_roi_data;   /*Simple ROI data packet @see SimpleRoiDataTypeDef*/
    FullRoiDataTypeDef* full_roi_data;       /*Full ROI data packet @see FullRoiDataTypeDef*/
    DepthDataTypeDef* simple_depth_data;     /*Simple depth data packet  @see DepthDataTypeDef*/
    DepthDataTypeDef* full_depth_data;       /*Full ROI data packet @see DepthDataTypeDef*/
    PointCloudDataTypeDef* point_cloud_data; /*Point cloud data packet @see PointCloudDataTypeDef*/
    ObstacleDataTypedef* Obstacle_data;      /*Obstacle data packet @see ObstacleDataTypedef*/
    uint8_t* Obstacle_frame_data_buff;       /*Buffer used to store obstacle data packet*/
} MeasureDataTypeDef;

typedef enum {
    SYNC = 0x01, /*Synchronous mode*/
    ASYNC = 0x02 /*Asynchronous mode*/
} HPS3D_SynchronousTypedef;

/*Transport Type*/
typedef enum {
    TRANSPORT_USB = 0,
    TRANSPORT_CAN,
    TRANSPORT_RS232,
    TRANSPORT_RS485,
    TRANSPORT_ETHERNET
} TransportTypeDef;

/*handle*/
typedef struct {
    char* DeviceName;                  /*R/W Device name*/
    uint32_t DeviceFd;                 /*R   Not editable*/
    uint8_t DeviceAddr;                /*R   Store the device address (also the frame ID) of the
                                          currently connected device*/
    uint8_t ConnectionNumber;          /*R   Not editable*/
    HPS3D_SynchronousTypedef SyncMode; /*R   Synchronous or asynchronous mode*/
    RunModeTypeDef RunMode;            /*R/W Run mode*/
    MeasureDataTypeDef MeasureData;    /*R   Synchronous measurement data, when measured in
                                          asynchronous mode, the measurement results are not saved
                                          here (can be operated by the observer)*/
    RetPacketTypedef RetPacketType;    /*R   Synchronous measurement returns the packet type. When
                                          the asynchronous mode is measured, the result of
                                          returning the packet type will not be saved here (can be
                                          operated by the observer)*/
    OutPacketTypeDef OutputPacketType; /*R   Output packet type, not editable*/
    bool ConnectStatus;                /*R   Connect status, not editable*/
    uint8_t RoiNumber;                 /*R   Save the number of ROI supported by the device*/
    uint8_t ThresholdNumber;           /*R   Save the number of threshold supported by the
                                          device ROI*/
    uint8_t ViewAngleHorizontal;       /*R   Horizontal view of angle, not editable*/
    uint8_t ViewAngleVertical;         /*R   Vertical view of angle, no editable*/
    struct sockaddr_in ServerAddr;     /*R/W Sever ip address and port number*/
    TransportTypeDef TransportType;    /*R   当Current transport type*/
} HPS3D_HandleTypeDef;

/*Optical parameters*/
typedef struct {
    bool enable;                    /*Optical parameters enable(Turn on and the measured depth data
                                       is the vertical distance)*/
    uint8_t viewing_angle_horiz;    /*Horizontal view of angle*/
    uint8_t viewing_angle_vertical; /*Vertical view of angle*/
    uint8_t illum_angle_horiz;
    uint8_t illum_angle_vertical;
} OpticalParamConfTypeDef;

/*mult-devices mutual interference configuration, not editable*/
typedef struct {
    bool enable;
    uint32_t integ_time;
    uint16_t amplitude_threshold;
    uint8_t capture_num;
    uint8_t number_check;
} InterferenceDetectConfTypeDef;

/*Details for observer mode, please refer to
 * https://www.cnblogs.com/luohanguo/p/7825656.html
 * */
/*Observer subscription event*/
typedef enum {
    ISubject_Event_DataRecvd = 1 << 0,    /*Data receive event*/
    ISubject_Event_DevConnect = 1 << 1,   /*Connect even*/
    ISubject_Event_DevDisconnect = 1 << 2 /*Disconnect event*/
} AsyncISubjectEvent;

/*Observer subscription event structure parameters*/
typedef struct {
    uint8_t ObserverID;
    bool NotifyEnable;
    AsyncISubjectEvent AsyncEvent;  /*Observer subscription event*/
    MeasureDataTypeDef MeasureData; /*To store measure data*/
    RetPacketTypedef RetPacketType; /*Measure return packet type*/
} AsyncIObserver_t;

/*Point cloud data mirror*/
typedef enum {
    MORROR_DISABLE = 0X0,
    MIRROR_HORIZONTAL = 0X1,
    MIRROR_VERTICLA = 0X2,
} PointCloudImageMirrorTypeDef;

/**************************************Function
 * interface*************************************/

/***********************************1.Command function
 * interface***********************************/
/**
 * @brief ed mode
 * @param[in]
 * @note    Manually modify the handle->RunMode value before calling this
 * function， And this function works only after the device initialization is
 * completed, @see HPS3D_ConfigInit
 * @retval RET_OK
 */
extern RET_StatusTypeDef HPS3D_SetRunMode(HPS3D_HandleTypeDef* handle);

/**
 * @brief  address
 * @param[out]  handle->DeviceAddr  Output device address
 * @note    Device address( also the frame ID)
 * @retval RET_OK
 */
extern RET_StatusTypeDef HPS3D_GetDevAddr(HPS3D_HandleTypeDef* handle);

/**
 * @brief  address
 * @param[in] viceAddr   Old device address
 * @param[in]     s
 * @note evices can be distinguished by modifying this
 * parameter
 * @retval RET_OK
 */
extern RET_StatusTypeDef HPS3D_SetDevAddr(HPS3D_HandleTypeDef* handle, uint8_t new_addr);

/**
 * @brief  version information
 * @param[out]  version_t
 * @note  firmware version information
 * @retval RET_OK
 */
extern RET_StatusTypeDef HPS3D_GetDeviceVersion(HPS3D_HandleTypeDef* handle, Version_t* version_t);

/**
 * @brief er data packet type
 * @param[in] ote guration needs to be done before @see HPS3D_ConfigInit is
 * initialized. The default is full depth packet
 * @see
 * @code
 * @retval RET_OK
 */
extern RET_StatusTypeDef HPS3D_SetMeasurePacketType(MeasurePacketTypeDef type);
/**
 * @brief e data packet type
 * @param
 * @note  full depth data packet
 * @see f
 * @code
 * @retval e enumeration value
 */
extern MeasurePacketTypeDef HPS3D_GetMeasurePacketType(void);

/**
 * @brief e data return packet type(simple or full packet)
 * @param[in] outPacketType
 * @note    Set output data packet as simple data packet or full data packet
 * @see  @retval RET_OK
 */
extern RET_StatusTypeDef HPS3D_SetPacketType(HPS3D_HandleTypeDef* handle, OutPacketTypeDef outPacketType);

/**
 * @brief acket type
 * @param[out] cketType
 * @note
 * @see  @retval RET_OK
 */
extern RET_StatusTypeDef HPS3D_GetPacketType(HPS3D_HandleTypeDef* handle);

/**
 * @brief unication configuration
 * @note ion parameters will take effect permanently after using
 * this function, please keep in mind the current configuration (use with
 * caution)
 * @see
 * @code
 * @retval RET_OK
 */
extern RET_StatusTypeDef HPS3D_SaveTransportConf(HPS3D_HandleTypeDef* handle);

/**
 * @brief er configuration parameters
 * @param
 * @note ion will make the current configuration permanent, and
 * you can reset to the default configuration by clearing the configuration or
 * reset factor setting
 * @see urrent HPS3D_ProfileRestoreFactory
 * @retval RET_OK
 */
extern RET_StatusTypeDef HPS3D_ProfileSaveToCurrent(HPS3D_HandleTypeDef* handle);

/**
 * @brief  configuration parameters
 * @param
 * @note
 * @retval RET_OK
 */
extern RET_StatusTypeDef HPS3D_ProfileClearCurrent(HPS3D_HandleTypeDef* handle);

/**
 * @brief ory setting
 * @param
 * @note
 * @retval RET_OK
 */
extern RET_StatusTypeDef HPS3D_ProfileRestoreFactory(HPS3D_HandleTypeDef* handle);

/**
 * @brief ort type
 * @param[out] type
 * @note
 * @see ypeDef
 * @retval RET_OK
 * @see RET_StatusTypeDef
 */
extern RET_StatusTypeDef HPS3D_GetTransportType(HPS3D_HandleTypeDef* handle, TransportTypeDef* transport_type);

/**
 * @brief  group
 * @param[in] * @note
 * @retval RET_OK
 * @see RET_StatusTypeDef
 */
extern RET_StatusTypeDef HPS3D_SelectROIGroup(HPS3D_HandleTypeDef* handle, uint8_t group_id);

/**
 * @brief t ROI group ID
 * @param[out]  group_id
 * @note
 * @retval  Return OK RET_OK
 * @see RET_StatusTypeDef
 */
extern RET_StatusTypeDef HPS3D_GetROIGroupID(HPS3D_HandleTypeDef* handle, uint8_t* group_id);

/**
 * @brief arm type
 * @param[in]
 * @see ROIAlarmTypeDef
 * @note
 * @retval RET_OK @see RET_StatusTypeDef
 */
extern RET_StatusTypeDef HPS3D_SetROIAlarmType(HPS3D_HandleTypeDef* handle, uint8_t roi_id, uint8_t threshold_id,
                                               ROIAlarmTypeDef roi_alarm_type);

/**
 * @brief ference type
 * @param[in]
 * @see ROIReferenceTypeDef
 * @note
 * @retval RET_OK @see RET_StatusTypeDef
 */
extern RET_StatusTypeDef HPS3D_SetROIReferenceType(HPS3D_HandleTypeDef* handle, uint8_t roi_id, uint8_t threshold_id,
                                                   ROIReferenceTypeDef ref_type);

/**
 * @brief ea
 * @param[in] * @see ROIConfTypeDef
 * @note
 * @retval RET_OK @see RET_StatusTypeDef
 */
extern RET_StatusTypeDef HPS3D_SetROIRegion(HPS3D_HandleTypeDef* handle, ROIConfTypeDef roi_conf);

/**
 * @brief able
 * @param[in] @param[in] e
 * @retval RET_OK @see RET_StatusTypeDef
 */
extern RET_StatusTypeDef HPS3D_SetROIEnable(HPS3D_HandleTypeDef* handle, uint32_t roi_id, bool en);

/**
 * @brief reshold enable
 * @param[in] @param[in] id
 * @param[in] e
 * @retval RET_OK @see RET_StatusTypeDef
 */
extern RET_StatusTypeDef HPS3D_SetROIThresholdEnable(HPS3D_HandleTypeDef* handle, uint32_t roi_id,
                                                     uint32_t threshold_id, bool en);

/**
 * @brief reshold configuration
 * @param[in] _conf
 * @see HysteresisSingleConfTypeDef
 * @note
 * @retval RET_OK @see RET_StatusTypeDef
 */
extern RET_StatusTypeDef HPS3D_SetROIThresholdConf(HPS3D_HandleTypeDef* handle, uint32_t roi_id, uint32_t threshold_id,
                                                   uint16_t pix_num_threshold,
                                                   HysteresisSingleConfTypeDef hysteresis_conf);

/**
 * @brief mber of ROIs and thresholds supported by the current
 * device
 * @param[out]
 * @param[out] number
 * @note
 * @retval RET_OK @see RET_StatusTypeDef
 */
extern RET_StatusTypeDef HPS3D_GetNumberOfROI(HPS3D_HandleTypeDef* handle, uint8_t* roi_number,
                                              uint8_t* threshold_number);

/**
 * @brief ecified ROI configuration
 * @param[out] * @see ROIConfTypeDef
 * @note
 * @retval RET_OK @see RET_StatusTypeDef
 */
extern RET_StatusTypeDef HPS3D_GetROIConfById(HPS3D_HandleTypeDef* handle, uint8_t roi_id, ROIConfTypeDef* roi_conf);

/**
 * @brief nfiguration of the specified GPIO output port
 * @param[in] onf
 * @see GPIOOutConfTypeDef
 * @note        gpio_out_conf：Can only configure IO output
 * @retval RET_OK
 */
extern RET_StatusTypeDef HPS3D_SetGPIOOut(HPS3D_HandleTypeDef* handle, GPIOOutConfTypeDef gpio_out_conf);

/**
 * @brief nfiguration of the specified GPIO output port
 * @param[in] onf
 * @see GPIOOutConfTypeDef
 * @note
 * @retval RET_OK
 */
extern RET_StatusTypeDef HPS3D_GetGPIOOutConf(HPS3D_HandleTypeDef* handle, GPIOOutConfTypeDef* gpio_out_conf);

/**
 * @brief nfiguration of the specified GPIO input port
 * @param[in] nf
 * @see GPIOInConfTypeDef
 * @note nf：Can only configure IO input
 * @retval RET_OK
 */
extern RET_StatusTypeDef HPS3D_SetGPIOIn(HPS3D_HandleTypeDef* handle, GPIOInConfTypeDef gpio_in_conf);

/**
 * @brief nfiguration of the specified GPIO input port
 * @param[out] nf
 * @see GPIOInConfTypeDef
 * @note
 * @retval  RET_OK
 */
extern RET_StatusTypeDef HPS3D_GetGPIOInConf(HPS3D_HandleTypeDef* handle, GPIOInConfTypeDef* gpio_in_conf);

/**
 * @brief de
 * @param[in] * @see HDRModeTypeDef
 * @note
 * @retval RET_OK
 */
extern RET_StatusTypeDef HPS3D_SetHDRMode(HPS3D_HandleTypeDef* handle, HDRModeTypeDef hdr_mode);

/**
 * @brief  @param[in] hdr_conf
 * @see   @note
 * @retval RET_OK
 */
extern RET_StatusTypeDef HPS3D_SetHDRConfig(HPS3D_HandleTypeDef* handle, HDRConf hdr_conf);

/**
 * @brief nfiguration
 * @param[out] hdr_conf
 * @see  @retval  RET_OK
 */
extern RET_StatusTypeDef HPS3D_GetHDRConfig(HPS3D_HandleTypeDef* handle, HDRConf* hdr_conf);

/**
 * @brief stance filter type
 * @param[in] ilter_conf
 * @see lterTypeDef
 * @note
 * @retval RET_OK
 */
extern RET_StatusTypeDef HPS3D_SetDistanceFilterType(HPS3D_HandleTypeDef* handle,
                                                     DistanceFilterTypeDef distance_filter_conf);

/**
 * @brief e distance filter
 * @param[in] ilter_conf
 * @see DistanceFilterConfTypeDef
 * @note
 * @retval RET_OK
 */
extern RET_StatusTypeDef HPS3D_SetSimpleKalman(HPS3D_HandleTypeDef* handle,
                                               DistanceFilterConfTypeDef distance_filter_conf);

/**
 * @brief ce filter configuration
 * @param[out] ilter_conf
 * @see ypeDef
 * @note
 * @retval RET_OK
 */
extern RET_StatusTypeDef HPS3D_GetDistanceFilterConf(HPS3D_HandleTypeDef* handle,
                                                     DistanceFilterConfTypeDef* distance_filter_conf);

/**
 * @brief ooth filter
 * @param[in] ter_conf
 * @see erConfTypeDef
 * @note
 * @retval RET_OK
 */
extern RET_StatusTypeDef HPS3D_SetSmoothFilter(HPS3D_HandleTypeDef* handle, SmoothFilterConfTypeDef smooth_filter_conf);

/**
 * @brief nfiguration of the smooth filter
 * @param[out] ter_conf
 * @see erConfTypeDef
 * @note
 * @retval RET_OK
 */
extern RET_StatusTypeDef HPS3D_GetSmoothFilterConf(HPS3D_HandleTypeDef* handle,
                                                   SmoothFilterConfTypeDef* smooth_filter_conf);

/**
 * @brief l parameter enable
 * @param[in] e rameter compensation is to convert the actual optical
 * path into a horizontal distance
 * @retval RET_OK
 */
extern RET_StatusTypeDef HPS3D_SetOpticalEnable(HPS3D_HandleTypeDef* handle, bool en);

/**
 * @brief l parameters
 * @param[out] ram_conf
 * @see amConfTypeDef
 * @note
 * @retval RET_OK
 */
extern RET_StatusTypeDef HPS3D_GetOpticalParamConf(HPS3D_HandleTypeDef* handle,
                                                   OpticalParamConfTypeDef* optical_param_conf);

/**
 * @brief ce compensation
 * @note
 * @retval RET_OK
 */
extern RET_StatusTypeDef HPS3D_SetDistanceOffset(HPS3D_HandleTypeDef* handle, int16_t offset);

/**
 * @brief ce compensation
 * @param[out] @note
 * @retval RET_OK
 */
extern RET_StatusTypeDef HPS3D_GetDistanceOffset(HPS3D_HandleTypeDef* handle, int16_t* offset);

/**
 * @brief devices multual interference detection enable
 * @param[in] e
 * @retval RET_OK
 */
extern RET_StatusTypeDef HPS3D_SetInterferenceDetectEn(HPS3D_HandleTypeDef* handle, bool en);

/**
 * @brief stallation angle change enable
 * @param[in] e
 * @retval RET_OK
 */
/*extern RET_StatusTypeDef HPS3D_SetMountingAngleEnable(HPS3D_HandleTypeDef
 * *handle, bool en);*/ /*This interface is no longer used and is replaced by the following interface*/

/**
 * @brief stallation angle transformation parameters
 * @param[in] ngle_param_conf
 * @see  ypeDef
 * @note
 * @retval RET_OK
 */
extern RET_StatusTypeDef HPS3D_SetMountingAngleParamConf(HPS3D_HandleTypeDef* handle,
                                                         MountingAngleParamTypeDef mounting_angle_param_conf);

/**
 * @brief stallation angle transformation parameters
 * @param[out] ngle_param_conf
 * @see  ypeDef
 * @note
 * @retval RET_OK
 */
extern RET_StatusTypeDef HPS3D_GetMountingParamConf(HPS3D_HandleTypeDef* handle,
                                                    MountingAngleParamTypeDef* mounting_angle_param_conf);

/*************************************2.Integrated function
 * interface**********************************/

/**
 * @brief ecified prefix file under the directory (automatically
 * find the device))
 * @param[in] vice file root directory
 * @param[in]   prefix  Device file name prefix
 * @param[out]  fileName To save the device found in the current current
 * directory
 * @note HPS3D_GetDeviceList("/dev/","ttyACM",fileName);
 * @retval  number of successful acquisitions 0 indicates failure
 */
extern uint32_t HPS3D_GetDeviceList(char* dirPath, char* prefix, char fileName[DEV_NUM][DEV_NAME_SIZE]);

/**
 * @brief nect
 * @param[out] nnectStatus
 * @note
 * @retval RET_OK
 */
extern RET_StatusTypeDef HPS3D_Connect(HPS3D_HandleTypeDef* handle);

/**
 * @brief
 * @param[out] nnectStatus
 * @note
 * @retval RET_OK
 */
extern RET_StatusTypeDef HPS3D_DisConnect(HPS3D_HandleTypeDef* handle);

/**
 * @brief tialization
 * @param[in]
 * @note hread and get the sensor initialization parameters to
 * apply for memory space; If need to use, @see
 * HPS3D_AutoConnectAndInitConfigDevice
 * @retval RET_OK
 */
extern RET_StatusTypeDef HPS3D_ConfigInit(HPS3D_HandleTypeDef* handle);

/**
   * @brief set serial port properties
   * @param[in] uartConfig
   * @note
   * @see UartConfTypeDef
   * @code
   * @retval successfully returns RET_OK
   */
RET_StatusTypeDef HPS3D_SetUartConfig(UartConfTypeDef uartConfig);

/**
 * @brief reshold
 * @param[in] id
 * @param[in] * @see ROIConfTypeDef
 * @retval RET_OK
 */
extern RET_StatusTypeDef HPS3D_SetThreshold(HPS3D_HandleTypeDef* handle, uint8_t threshold_id, ROIConfTypeDef roi_conf);

/**
 * @brief rameters
 * @param[in]   roi_conf
 * @see ROIConfTypeDef  GPIOOutConfTypeDef
 * @retval RET_OK
 */
extern RET_StatusTypeDef HPS3D_SetSingleRoi(HPS3D_HandleTypeDef* handle, ROIConfTypeDef roi_conf,
                                            GPIOOutConfTypeDef gpio_out_conf);

/**
 * @brief er(registrate callback function notification event)
 * @param[in]
 * @see AsyncIObserver_t
 * @note  us mode (continuous measurement mode uses this mode)
 * @retval  Return OK RET_OK
 */
extern RET_StatusTypeDef HPS3D_AddObserver(void* (*fun)(HPS3D_HandleTypeDef*, AsyncIObserver_t*),
                                           HPS3D_HandleTypeDef* handle, AsyncIObserver_t* Observer_t);

/**
 * @brief server (logout notification event)
 * @param[in]
 * @see  @note us mode (continuous measurement mode uses this mode)
 * @retval Return OK RET_OK
 */
RET_StatusTypeDef HPS3D_RemoveObserver(AsyncIObserver_t* Observer_t);

/**
 * @brief nstallation and resource recycle
 * @param[in]
 * @note
 * @retval Return OK RET_OK
 */
extern RET_StatusTypeDef HPS3D_RemoveDevice(HPS3D_HandleTypeDef* handle);

/**
 * @brief  @param[in] e
 * @retval  RET_OK
 */
extern RET_StatusTypeDef HPS3D_SetDebugEnable(bool en);

/**
 * @brief enable status
 * @param
 * @note
 * @retval ug enable status value
 */
extern bool HPS3D_GetDebugEnable(void);

/**
 * @brief  of the callback function
 * @param[in] _Back     Receive callback function address callback
 * function is void *fun(uint8_t *str, uint16_t *str_len){...}
 * @param[out] callback function str和strlen
 * @note
 * @retval  RET_OK
 */
RET_StatusTypeDef HPS3D_SetDebugFunc(void (*Call_Back)(char* str));

/**
 * @brief cloud data conversion enable
 * @param[in] e
 * @retval RET_OK
 */
extern RET_StatusTypeDef HPS3D_SetPointCloudEn(bool en);

/**
 * @brief cloud data conversion enable status
 * @param
 * @note
 * @retval lse
 */
extern bool HPS3D_GetPointCloudEn(void);

/**
 * @brief  * @param[out] tPacketType ket type
 * @param[out] asureData te   This method is synchronous measurement, that is,
 * the measured return value is obtained immediately after calling this
 * function.
 * @see
 * @code
 * @retval  RET_OK
 */
extern RET_StatusTypeDef HPS3D_SingleMeasurement(HPS3D_HandleTypeDef* handle);

/**
 * @brief  extraction parameters
 * @param[in]
 * @note
 * @see ObstacleConfigTypedef
 * @code
 *
 * @retval  RET_OK
 */
extern ObstacleConfigTypedef HPS3D_GetObstacleConfigInit(void);

/**
 * @brief tion configuration parameters
 * @param
 * @note
 * @see ObstacleConfigTypedef
 * @code
 * @retval RET_OK
 */
extern RET_StatusTypeDef HPS3D_ObstacleConfigInit(ObstacleConfigTypedef Conf);

/**
 * @brief umber of obstacle pixels
 * @param
 * @note  Set this threshold to filter small obstacles.
 * @see
 * @code
 * @retval none
 */
extern void HPS3D_SetObstaclePixelNumberThreshold(uint32_t pixel_num_thr);

/**
 * @brief umber of obstacle pixels
 * @param
 * @note
 * @see
 * @code
 * @retval  Return pixel threshold
 */
extern uint32_t HPS3D_GetObstaclePixelNumberThreshold(void);

/**
 * @brief le extraction threshold offset
 * @param
 * @note
 * @see
 * @code
 * @retval  none
 */
extern void HPS3D_SetThresholdOffset(int32_t thr_offset);

/**
 * @brief le extraction threshold offset
 * @param
 * @note
 * @see
 * @code
 * @retval  Return current threshold
 */
extern int32_t HPS3D_GetThresholdOffset(void);

/**
 * @brief rsion
 * @param
 * @note
 * @see
 * @code
 * @retval nformation
 */
extern Version_t HPS3D_GetSDKVersion(void);

/**
 * @brief ation on how to convert special measured
 * output values to specified special value param[in] am[in] value
 * @note
 * @see
 * @code
 * @retval  RET_OK
 */
extern RET_StatusTypeDef HPS3D_ConfigSpecialMeasurementValue(bool enable, uint16_t value);

/**
 * @brief er enable
 * param[in]
 * @note
 * @see
 * @code
 * @retval RET_OK
 */
extern RET_StatusTypeDef HPS3D_SetEdgeDetectionEnable(bool en);

/**
 * @brief er enable
 * param[in]
 * @note
 * @see
 * @code
 *
 * @retval atus value
 */
extern bool HPS3D_GetEdgeDetectionEnable(void);

/**
 * @brief shold
 * param[in]
 * @note
 * @see
 * @code
 *
 * @retval  RET_OK
 */
extern RET_StatusTypeDef HPS3D_SetEdgeDetectionValue(int32_t threshold_value);

/**
 * @brief oise threshold
 * param[in]
 * @note
 * @see
 * @code
 *
 * @retval rrent threshold value
 */
extern int32_t HPS3D_GetEdgeDetectionValue(void);

/**
 * @brief  cloud data to ply format file
 * @param[in]
 * @param[in] point_cloud_data
 * @note
 * @see PointCloudDataTypeDef
 * @code
 * @retval Return OK RET_OK
 */
extern RET_StatusTypeDef HPS3D_SavePlyFile(char* filename, PointCloudDataTypeDef point_cloud_data);

/**
 * @brief stacle avoidance scheme (3IO version output alarm value
 * by IO)
 * @param[in] MeasureData
 * @param[in] AvoidConf
 * @note
 * @see AvoidanceTypeDef
 * @code
 * @retval RET_OK
 */
extern RET_StatusTypeDef HPS3D_ObstacleAvoidance(HPS3D_HandleTypeDef* handle, MeasureDataTypeDef* MeasureData,
                                                 AvoidanceTypeDef* AvoidConf);

/**
 * @brief cloud mirror
 * @param[in] type Mirror direction (horizontal or vertical)
 * @note
 * @see PointCloudImageMirrorTypeDef
 * @code
 * @retval RET_OK
 */
extern RET_StatusTypeDef HPS3D_SetPointCloudMirrorType(PointCloudImageMirrorTypeDef type);
/**
 * @brief cloud mirror
 * @param[in]
 * @note
 * @see PointCloudImageMirrorTypeDef
 * @code
 * @retval PointCloudImageMirrorTypeDef
 */
extern PointCloudImageMirrorTypeDef HPS3D_GetPointCloudMirrorType(void);

/**
 * @brief  information
 * @param[in]
 * @note eg: char *serverIP = "192.168.0.10";
 *         serverPort = 12345;  This function needs to be called before connect
 * @see
 * @code
 * @retval RET_OK
 */
extern RET_StatusTypeDef HPS3D_SetEthernetServerInfo(HPS3D_HandleTypeDef* handle, char* serverIP, uint16_t serverPort);

/**
 * @brief er IP（To edit device default IP. Edit here is not
 * recommended
 * @param[in]  serverIP
 * @param[in]  serverPort
 * @param[in]  netmask
 * @param[in]  gateway
 * @note eg:serverIP[4] = {192,168,0,10};
 *         netMask[4] = {255,255,255,0};
 *         geteway[4] = {192,168,0,1};
 *         serverPort = 12345;
 * @see
 * @code
 * @retval RET_OK
 */
RET_StatusTypeDef HPS3D_ConfigEthernet(HPS3D_HandleTypeDef* handle, uint8_t* serverIP, uint16_t serverPort,
                                       uint8_t* netMask, uint8_t* geteway);

/**
 * @brief evices automatically connect and initialize (the number
 * of connections is limited by DEV_NUM and can be modified)
 * @param[in]
 * @note
 * @see
 * @code
 * @retval  number of successful connections
 */
extern uint8_t HPS3D_AutoConnectAndInitConfigDevice(HPS3D_HandleTypeDef* handle);

/**
  * @brief set multi-machine encoding
  * @param[in] CameraCode Multi-machine code
  * @note Multi-machine encoding only supports firmware version 1.7.62 and
above, and the code serial number is 0-15.   * @see   * @code   *   * @retval
Return OK RET_OK   */
extern RET_StatusTypeDef HPS3D_SetMultiCameraCode(HPS3D_HandleTypeDef* handle, uint8_t CameraCode);

/**
  * @brief Get multi-machine code
  * @param[in] handle
  * @note
  * @see
  * @code
  *
  * @retval returns the current multi-machine code value
  */
extern uint8_t HPS3D_GetMultiCameraCode(HPS3D_HandleTypeDef* handle);

/**
  * @brief set heartbeat detection
  * @param[in] handle
  * @param[in] enable Heartbeat Detection Enable
  * @param[in] time_ms heartbeat detection time ms
  * @note After calling this interface, send a heartbeat packet
HPS3D_SendKeepAlive before HPS3D_SetRunMode
  * @see
  * @code
  * @retval RET_OK
  */
extern RET_StatusTypeDef HPS3D_SetKeepAliveConfig(HPS3D_HandleTypeDef* handle, bool enable, uint32_t time_ms);

/**
  * @brief sends a keep-alive command
  * @param[in] handle
  * @note This interface needs to be sent periodically within the set heartbeat
detection time. After sending this command, it is only necessary to detect
whether there is a heartbeat return packet within the set time.   *
KEEP_ALIVE_PACKET in @see RetPacketTypedef
  * @code
  * @retval RET_OK
  */
extern RET_StatusTypeDef HPS3D_SendKeepAlive(HPS3D_HandleTypeDef* handle);

#ifdef __cplusplus
}
#endif

#endif /* API_H_ */
