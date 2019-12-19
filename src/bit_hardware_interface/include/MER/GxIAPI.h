/**
@File      GxIAPI.h
@Brief     the interface for the GxIAPI dll module. 
@Author    Software Department
@Date      2019-4-26
@Version   1.0.1904.9261
@Copyright Copyright(C)2012 Daheng Imavision
*/

#ifndef GX_GALAXY_H
#define GX_GALAXY_H


//////////////////////////////////////////////////////////////////////////
//	类型定义，以下类型都在标准C库头文件stdint.h中有定义，但是在微软的编译平台
//	VS2010之前的版本中都不包含此文件,所以在此需要重定义
//////////////////////////////////////////////////////////////////////////

#if defined(_WIN32)
	#ifndef _STDINT_H 
		#ifdef _MSC_VER // Microsoft compiler
			#if _MSC_VER < 1600
				typedef __int8            int8_t;
				typedef __int16           int16_t;
				typedef __int32           int32_t;
				typedef __int64           int64_t;
				typedef unsigned __int8   uint8_t;
				typedef unsigned __int16  uint16_t;
				typedef unsigned __int32  uint32_t;
				typedef unsigned __int64  uint64_t;
			#else
				// In Visual Studio 2010 is stdint.h already included
				#include <stdint.h>
			#endif
		#else
			// Not a Microsoft compiler
			#include <stdint.h>
		#endif
	#endif 
#else
	// Linux
	#include <stdint.h>
#endif


//------------------------------------------------------------------------------
//  操作系统平台定义
//------------------------------------------------------------------------------

#include <stddef.h>

#ifdef WIN32
	#ifndef _WIN32
		#define _WIN32
	#endif
#endif

#ifdef _WIN32
	#include <Windows.h>
	#define GX_DLLIMPORT   __declspec(dllimport)
	#define GX_DLLEXPORT   __declspec(dllexport)

	#define GX_STDC __stdcall
	#define GX_CDEC __cdecl

	#if defined(__cplusplus)
		#define GX_EXTC extern "C"
	#else
		#define GX_EXTC
	#endif
#else
	// remove the None #define conflicting with GenApi
	#undef None
	#if __GNUC__>=4
		#define GX_DLLIMPORT   __attribute__((visibility("default")))
		#define GX_DLLEXPORT   __attribute__((visibility("default")))

		#if defined(__i386__)
			#define GX_STDC __attribute__((stdcall))
			#define GX_CDEC __attribute__((cdecl))
		#else
			#define GX_STDC 
			#define GX_CDEC 
		#endif

		#if defined(__cplusplus)
			#define GX_EXTC extern "C"
		#else
			#define GX_EXTC
		#endif
	#else
		#error Unknown compiler
	#endif
#endif

#ifdef GX_GALAXY_DLL
	#define GX_DLLENTRY GX_EXTC GX_DLLEXPORT
#else
	#define GX_DLLENTRY GX_EXTC GX_DLLIMPORT
#endif


//------------------------------------------------------------------------------
//  错误码定义
//------------------------------------------------------------------------------
typedef int32_t GX_STATUS;

typedef enum GX_STATUS_LIST
{
	GX_STATUS_SUCCESS            = 0,            ///< 成功
	GX_STATUS_ERROR              = -1,          ///< 失败
	GX_STATUS_INVALID_HANDLE     = -6,          ///< 非法句柄
	GX_STATUS_INVALID_PARAMETER  = -5,          ///< 参数非法
	GX_STATUS_OUT_RANGE          = -11,         ///< 越界错误
	GX_STATUS_TIMEOUT_ERROR      = -14,         ///< 超时错误
}GX_STATUS_LIST;

//------------------------------------------------------------------------------
//  帧状态码定义
//------------------------------------------------------------------------------
typedef enum GX_FRAME_STATUS_LIST
{
    GX_FRAME_STATUS_SUCCESS             = 0,     ///< 正常帧
    GX_FRAME_STATUS_INCOMPLETE          = -1,    ///< 残帧
    GX_FRAME_STATUS_INVALID_IMAGE_INFO  = -2,    ///< 信息错误帧
}GX_FRAME_STATUS_LIST;
typedef  int32_t  GX_FRAME_STATUS;

//------------------------------------------------------------------------------
//  设备类型码定义
//------------------------------------------------------------------------------
typedef enum GX_DEVICE_CLASS_LIST
{
	GX_DEVICE_CLASS_UNKNOWN = 0,     ///< 未知设备类型
	GX_DEVICE_CLASS_USB2    = 1,     ///< USB2.0设备
	GX_DEVICE_CLASS_GEV     = 2,     ///< 千兆网设备
}GX_DEVICE_CLASS_LIST;
typedef  int32_t GX_DEVICE_CLASS;

//------------------------------------------------------------------------------
//  功能码定义
//------------------------------------------------------------------------------
typedef enum GX_FEATURE_TYPE
{
	GX_FEATURE_INT				   =0x10000000,  ///< 整型数
	GX_FEATURE_FLOAT               =0X20000000,  ///< 浮点数
	GX_FEATURE_ENUM				   =0x30000000,  ///< 枚举
	GX_FEATURE_BOOL				   =0x40000000,  ///< 布尔
	GX_FEATURE_STRING			   =0x50000000,  ///< 字符串
	GX_FEATURE_BUFFER			   =0x60000000,  ///< buffer
	GX_FEATURE_COMMAND			   =0x70000000,  ///< 命令
}GX_FEATURE_TYPE;

typedef enum GX_FEATURE_ID
{
	//---------------DeviceInfomation Section--------------------------
	GX_STRING_DEVICE_VENDOR_NAME      = 0 | GX_FEATURE_STRING,     ///< 厂商名称
	GX_STRING_DEVICE_MODEL_NAME       = 1 | GX_FEATURE_STRING,     ///< 设备类型名称
	GX_STRING_DEVICE_FIRMWARE_VERSION = 2 | GX_FEATURE_STRING,     ///< 固件版本
	GX_STRING_DEVICE_VERSION          = 3 | GX_FEATURE_STRING,     ///< FPGA版本
	GX_STRING_DEVICE_ID               = 4 | GX_FEATURE_STRING,     ///< 序列号
	GX_STRING_DEVICE_HARDWARE_VERSION = 5 | GX_FEATURE_STRING,     ///< 硬件版本

	//---------------ImageFormat Section--------------------------------
	GX_INT_SENSOR_WIDTH               = 1000 | GX_FEATURE_INT,     ///< 传感器宽度
	GX_INT_SENSOR_HEIGHT              = 1001 | GX_FEATURE_INT,     ///< 传感器高度
	GX_INT_WIDTH_MAX                  = 1002 | GX_FEATURE_INT,     ///< 最大宽度
	GX_INT_HEIGHT_MAX                 = 1003 | GX_FEATURE_INT,     ///< 最大高度
	GX_INT_OFFSET_X                   = 1004 | GX_FEATURE_INT,     ///< X方向偏移
	GX_INT_OFFSET_Y                   = 1005 | GX_FEATURE_INT,     ///< Y方向偏移
	GX_INT_WIDTH                      = 1006 | GX_FEATURE_INT,     ///< 图像宽度
	GX_INT_HEIGHT                     = 1007 | GX_FEATURE_INT,     ///< 图像高度
	GX_INT_BINNING_HORIZONTAL         = 1008 | GX_FEATURE_INT,     ///< 水平合并
	GX_INT_BINNING_VERTICAL           = 1009 | GX_FEATURE_INT,     ///< 垂直合并
	GX_INT_DECIMATION_HORIZONTAL      = 1010 | GX_FEATURE_INT,     ///< 水平抽取
	GX_INT_DECIMATION_VERTICAL        = 1011 | GX_FEATURE_INT,     ///< 垂直抽取
	GX_ENUM_PIXEL_SIZE                = 1012 | GX_FEATURE_ENUM,    ///< 像素点大小,参考GX_PIXEL_SIZE_ENTRY
	GX_ENUM_PIXEL_COLOR_FILTER        = 1013 | GX_FEATURE_ENUM,    ///< Bayer格式,参考GX_PIXEL_COLOR_FILTER_ENTRY
	GX_ENUM_PIXEL_FORMAT              = 1014 | GX_FEATURE_ENUM,    ///< 数据格式,参考GX_PIXEL_FORMAT_ENTRY
	GX_BOOL_REVERSE_X                 = 1015 | GX_FEATURE_BOOL,    ///< 水平翻转使能
	GX_BOOL_REVERSE_Y                 = 1016 | GX_FEATURE_BOOL,    ///< 垂直翻转使能

	//---------------TransportLayer Section-------------------------------
	GX_INT_PAYLOAD_SIZE               = 2000 | GX_FEATURE_INT,     ///< 图像数据大小 

	//---------------AcquisitionTrigger Section---------------------------
	GX_ENUM_ACQUISITION_MODE          = 3000 | GX_FEATURE_ENUM,    ///< 采集模式,参考GX_ACQUISITION_MODE_ENTRY
	GX_COMMAND_ACQUISITION_START      = 3001 | GX_FEATURE_COMMAND, ///< 开采命令
	GX_COMMAND_ACQUISITION_STOP       = 3002 | GX_FEATURE_COMMAND, ///< 停采命令
	GX_INT_ACQUISITION_SPEED_LEVEL    = 3003 | GX_FEATURE_INT,     ///< 采集速度级别
	GX_INT_ACQUISITION_FRAME_COUNT    = 3004 | GX_FEATURE_INT,     ///< 采多帧模式下的帧数
	GX_ENUM_TRIGGER_MODE              = 3005 | GX_FEATURE_ENUM,    ///< 触发模式,参考GX_TRIGGER_MODE_ENTRY
	GX_COMMAND_TRIGGER_SOFTWARE       = 3006 | GX_FEATURE_COMMAND, ///< 软触发命令
	GX_ENUM_TRIGGER_ACTIVATION        = 3007 | GX_FEATURE_ENUM,    ///< 触发极性,参考GX_TRIGGER_ACTIVATION_ENTRY
	GX_ENUM_TRIGGER_SWITCH            = 3008 | GX_FEATURE_ENUM,    ///< 触发开关,参考GX_TRIGGER_SWITCH_ENTRY
	GX_FLOAT_EXPOSURE_TIME            = 3009 | GX_FEATURE_FLOAT,   ///< 曝光时间
	GX_ENUM_EXPOSURE_AUTO             = 3010 | GX_FEATURE_ENUM,    ///< 自动曝光开关,参考GX_EXPOSURE_AUTO_ENTRY
	GX_FLOAT_TRIGGER_FILTER_RAISING   = 3011 | GX_FEATURE_FLOAT,   ///< 触发滤波，上升沿
	GX_FLOAT_TRIGGER_FILTER_FALLING   = 3012 | GX_FEATURE_FLOAT,   ///< 触发滤波，下降沿

	//----------------DigitalIO Section----------------------------------
	GX_ENUM_USER_OUTPUT_SELECTOR      = 4000 | GX_FEATURE_ENUM,   ///< 用户输出源选择,参考GX_USER_OUTPUT_SELECTOR_ENTRY
	GX_BOOL_USER_OUTPUT_VALUE         = 4001 | GX_FEATURE_BOOL,   ///< 用户输出极性
	GX_ENUM_USER_OUTPUT_MODE          = 4002 | GX_FEATURE_ENUM,   ///< 用户输出模式选择,参考GX_USER_OUTPUT_MODE_ENTRY
	GX_ENUM_STROBE_SWITCH             = 4003 | GX_FEATURE_ENUM,   ///< 闪光灯模式开关,参考GX_STROBE_SWITCH_ENTRY

	//----------------AnalogControls Section----------------------------
	GX_ENUM_GAIN_AUTO                 = 5000 | GX_FEATURE_ENUM,   ///< 自动增益开关,参考GX_GAIN_AUTO_ENTRY
	GX_ENUM_GAIN_SELECTOR             = 5001 | GX_FEATURE_ENUM,   ///< 增益通道选择,参考GX_GAIN_SELECTOR_ENTRY
	GX_INT_GAIN                       = 5002 | GX_FEATURE_INT,    ///< 增益
	GX_ENUM_BLACKLEVEL_AUTO           = 5003 | GX_FEATURE_ENUM,   ///< 自动黑电平开关,参考GX_BLACKLEVEL_AUTO_ENTRY
	GX_ENUM_BLACKLEVEL_SELECTOR       = 5004 | GX_FEATURE_ENUM,   ///< 黑电平通道选择,参考GX_BLACKLEVEL_SELECTOR_ENTRY
	GX_INT_BLACKLEVEL                 = 5005 | GX_FEATURE_INT,    ///< 黑电平
	GX_ENUM_BALANCE_WHITE_AUTO        = 5006 | GX_FEATURE_ENUM,   ///< 自动白平衡开关,参考GX_BALANCE_WHITE_AUTO_ENTRY
	GX_FLOAT_BALANCE_RATIO_SELECTOR   = 5007 | GX_FEATURE_ENUM,   ///< 白平衡通道,参考GX_BALANCE_RATIO_SELECTOR_ENTRY[弃用]
	GX_ENUM_BALANCE_RATIO_SELECTOR    = 5007 | GX_FEATURE_ENUM,   ///< 白平衡通道,参考GX_BALANCE_RATIO_SELECTOR_ENTRY
	GX_FLOAT_BALANCE_RATIO            = 5008 | GX_FEATURE_FLOAT,  ///< 白平衡通道系数
	GX_ENUM_COLOR_CORRECT             = 5009 | GX_FEATURE_ENUM,   ///< 颜色校正开关,参考GX_COLOR_CORRECT_ENTRY
	GX_ENUM_DEAD_PIXEL_CORRECT        = 5010 | GX_FEATURE_ENUM,   ///< 自动坏点校正开关,参考GX_DEAD_PIXEL_CORRECT_ENTRY

	//---------------UserDefinedValues Section-------------------------
	GX_INT_ADC_LEVEL                  = 6000 | GX_FEATURE_INT,    ///< ADC级别
	GX_INT_H_BLANKING                 = 6001 | GX_FEATURE_INT,    ///< 水平消隐
	GX_INT_V_BLANKING                 = 6002 | GX_FEATURE_INT,    ///< 垂直消隐
	GX_STRING_USER_PASSWORD           = 6003 | GX_FEATURE_STRING, ///< 用户加密区密码
	GX_STRING_VERIFY_PASSWORD         = 6004 | GX_FEATURE_STRING, ///< 用户加密区校验密码
	GX_BUFFER_USER_DATA               = 6005 | GX_FEATURE_BUFFER, ///< 用户加密区内容
	GX_INT_GRAY_VALUE                 = 6006 | GX_FEATURE_INT,    ///< 期望灰度值
	GX_ENUM_AA_LIGHT_ENVIRMENT        = 6007 | GX_FEATURE_ENUM,   ///< 2A光照环境类型,参考GX_AA_LIGHT_ENVIRMENT_ENTRY
	GX_INT_ROI_X                      = 6008 | GX_FEATURE_INT,    ///< 感兴趣区域X偏移
	GX_INT_ROI_Y                      = 6009 | GX_FEATURE_INT,    ///< 感兴趣区域Y偏移
	GX_INT_ROI_WIDTH                  = 6010 | GX_FEATURE_INT,    ///< 感兴趣区域宽度
	GX_INT_ROI_HEIGHT                 = 6011 | GX_FEATURE_INT,    ///< 感兴趣区域高度
	GX_INT_AUTO_GAIN_VALUEMIN         = 6012 | GX_FEATURE_INT,    ///< 自动增益最小值
	GX_INT_AUTO_GAIN_VALUEMAX         = 6013 | GX_FEATURE_INT,    ///< 自动增益最大值
	GX_INT_AUTO_SHUTTER_VALUEMIN      = 6014 | GX_FEATURE_INT,    ///< 自动曝光最小值
	GX_INT_AUTO_SHUTTER_VALUEMAX      = 6015 | GX_FEATURE_INT,    ///< 自动曝光最大值
	GX_BUFFER_FRAME_INFORMATION       = 6016 | GX_FEATURE_BUFFER, ///< 图像帧信息

	//---------------UserSetControl Section-------------------------
	GX_ENUM_USER_SET_SELECTOR         = 7000 | GX_FEATURE_ENUM,    ///< 选择参数组,参考GX_USER_SET_SELECTOR_ENTRY
	GX_COMMAND_USER_SET_LOAD          = 7001 | GX_FEATURE_COMMAND, ///< 加载参数组
	GX_COMMAND_USER_SET_SAVE          = 7002 | GX_FEATURE_COMMAND, ///< 保存参数组

	//---------------Change type Section----------------------------
	GX_STRING_DEVICE_SERIAL_NUMBER    = 4   | GX_FEATURE_STRING,   ///< 设备序列号
	GX_FLOAT_GAIN                     = 5011 | GX_FEATURE_FLOAT,  ///< 增益
	GX_FLOAT_BLACKLEVEL               = 5012 | GX_FEATURE_FLOAT,  ///< 黑电平
	GX_ENUM_AA_LIGHT_ENVIRONMENT      = 6007 | GX_FEATURE_ENUM,   ///< 自动曝光、自动增益，光照环境类型
	GX_INT_AAROI_OFFSETX              = 6008 | GX_FEATURE_INT,    ///< 自动调节感兴趣区域X坐标
	GX_INT_AAROI_OFFSETY              = 6009 | GX_FEATURE_INT,    ///< 自动调节感兴趣区域Y坐标
	GX_INT_AAROI_WIDTH                = 6010 | GX_FEATURE_INT,    ///< 自动调节感兴趣区域宽度
	GX_INT_AAROI_HEIGHT               = 6011 | GX_FEATURE_INT,    ///< 自动调节感兴趣区域高度
	GX_FLOAT_AUTO_GAIN_MIN            = 6012 | GX_FEATURE_FLOAT,  ///< 自动增益最小值
	GX_FLOAT_AUTO_GAIN_MAX            = 6013 | GX_FEATURE_FLOAT,  ///< 自动增益最大值
	GX_FLOAT_AUTO_EXPOSURE_TIME_MIN   = 6014 | GX_FEATURE_FLOAT,  ///< 自动曝光最小值
	GX_FLOAT_AUTO_EXPOSURE_TIME_MAX   = 6015 | GX_FEATURE_FLOAT,  ///< 自动曝光最大值

}GX_FEATURE_ID;


//------------------------------------------------------------------------------
//  句柄定义
//------------------------------------------------------------------------------
typedef void* GX_DEV_HANDLE;           ///< 设备句柄

//------------------------------------------------------------------------------
//  枚举类型定义
//------------------------------------------------------------------------------

typedef enum GX_ACCESS_MODE//没有从0开始，是为了与TL定义一致。
{
	GX_ACCESS_NONE          =1,        ///< 
	GX_ACCESS_READONLY      =2,        ///< 只读方式
	GX_ACCESS_CONTROL       =3,        ///< 控制方式
	GX_ACCESS_EXCLUSIVE     =4,        ///< 独占方式
}GX_ACCESS_MODE;

//------------------------------------------------------------------------------
//  当前设备的可访问方式
//------------------------------------------------------------------------------
typedef enum GX_ACCESS_STATUS
{
	GX_ACCESS_STATUS_UNKNOWN    = 0,   ///< 设备当前状态未知
	GX_ACCESS_STATUS_READWRITE  = 1,   ///< 设备当前可读可写
	GX_ACCESS_STATUS_READONLY   = 2,   ///< 设备当前只支持读
	GX_ACCESS_STATUS_NOACCESS   = 3,   ///< 设备当前既不支持读，又不支持写
}GX_ACCESS_STATUS;
typedef int32_t GX_ACCESS_STATUS_CMD;

typedef enum GX_OPEN_MODE
{
	GX_OPEN_SN              =0,        ///< 通过SN打开
	GX_OPEN_IP              =1,        ///< 通过IP打开
	GX_OPEN_MAC             =2,        ///< 通过MAC打开
	GX_OPEN_INDEX           =3,        ///< 通过Index打开
}GX_OPEN_MODE;

typedef enum GX_UTI_CALLBACK_ID
{
	GX_UTICALLBACK_OFFLINE  = 0,       ///< 设备掉线通知回调,当系统检测到相机掉线后执行回调
}GX_UTI_CALLBACK_ID;
//------------------------------------------------------------------------------------

typedef enum GX_PIXEL_SIZE_ENTRY
{
	GX_PIXEL_SIZE_BPP8  = 8,
	GX_PIXEL_SIZE_BPP10 = 10,
	GX_PIXEL_SIZE_BPP12 = 12,
	GX_PIXEL_SIZE_BPP16 = 16,
	GX_PIXEL_SIZE_BPP24 = 24,
	GX_PIXEL_SIZE_BPP30 = 30,
	GX_PIXEL_SIZE_BPP32 = 32,
	GX_PIXEL_SIZE_BPP36 = 36,
	GX_PIXEL_SIZE_BPP48 = 48,
	GX_PIXEL_SIZE_BPP64 = 64,
}GX_PIXEL_SIZE_ENTRY;

typedef enum GX_PIXEL_COLOR_FILTER_ENTRY
{
	GX_COLOR_FILTER_NONE     = 0,                        ///<无
	GX_COLOR_FILTER_BAYER_RG = 1,                        ///<RG格式
	GX_COLOR_FILTER_BAYER_GB = 2,                        ///<GB格式
	GX_COLOR_FILTER_BAYER_GR = 3,                        ///<GR格式
	GX_COLOR_FILTER_BAYER_BG = 4,                        ///<BG格式
}GX_PIXEL_COLOR_FILTER_ENTRY;

#define GX_PIXEL_MONO                  ( 0x01000000 )
#define GX_PIXEL_COLOR                 ( 0x02000000 )

#define GX_PIXEL_8BIT                  ( 0x00080000 )
#define GX_PIXEL_10BIT                 ( 0x000A0000 )
#define GX_PIXEL_12BIT                 ( 0x000C0000 )
#define GX_PIXEL_16BIT                 ( 0x00100000 )
#define GX_PIXEL_24BIT                 ( 0x00180000 )
#define GX_PIXEL_30BIT                 ( 0x001E0000 )
#define GX_PIXEL_32BIT                 ( 0x00200000 )
#define GX_PIXEL_36BIT                 ( 0x00240000 )
#define GX_PIXEL_48BIT                 ( 0x00300000 )
#define GX_PIXEL_64BIT                 ( 0x00400000 )

typedef enum GX_PIXEL_FORMAT_ENTRY
{
	GX_PIXEL_FORMAT_UNDEFINED          = (0),
	GX_PIXEL_FORMAT_MONO8              = (GX_PIXEL_MONO  | GX_PIXEL_8BIT  | 0x0001),//0x1080001,
	GX_PIXEL_FORMAT_MONO8_SIGNED       = (GX_PIXEL_MONO  | GX_PIXEL_8BIT  | 0x0002),//0x1080002,
	GX_PIXEL_FORMAT_MONO10             = (GX_PIXEL_MONO  | GX_PIXEL_16BIT | 0x0003),//0x1100003,
	GX_PIXEL_FORMAT_MONO10_PACKED      = (GX_PIXEL_MONO  | GX_PIXEL_12BIT | 0x0004),//0x10c0004,
	GX_PIXEL_FORMAT_MONO12             = (GX_PIXEL_MONO  | GX_PIXEL_16BIT | 0x0005),//0x1100005,
	GX_PIXEL_FORMAT_MONO12_PACKED      = (GX_PIXEL_MONO  | GX_PIXEL_12BIT | 0x0006),//0x10c0006,
	GX_PIXEL_FORMAT_MONO14             = (GX_PIXEL_MONO  | GX_PIXEL_16BIT | 0x0025),//0x1100025,
	GX_PIXEL_FORMAT_MONO16             = (GX_PIXEL_MONO  | GX_PIXEL_16BIT | 0x0007),//0x1100007,
	GX_PIXEL_FORMAT_BAYER_GR8          = (GX_PIXEL_MONO  | GX_PIXEL_8BIT  | 0x0008),//0x1080008,               
	GX_PIXEL_FORMAT_BAYER_RG8          = (GX_PIXEL_MONO  | GX_PIXEL_8BIT  | 0x0009),//0x1080009,                
	GX_PIXEL_FORMAT_BAYER_GB8          = (GX_PIXEL_MONO  | GX_PIXEL_8BIT  | 0x000A),//0x108000A,
	GX_PIXEL_FORMAT_BAYER_BG8          = (GX_PIXEL_MONO  | GX_PIXEL_8BIT  | 0x000B),//0x108000B,
	GX_PIXEL_FORMAT_BAYER_GR10         = (GX_PIXEL_MONO  | GX_PIXEL_16BIT | 0x000C),//0x110000C,                
	GX_PIXEL_FORMAT_BAYER_RG10         = (GX_PIXEL_MONO  | GX_PIXEL_16BIT | 0x000D),//0x110000D,
	GX_PIXEL_FORMAT_BAYER_GB10         = (GX_PIXEL_MONO  | GX_PIXEL_16BIT | 0x000E),//0x110000E,
	GX_PIXEL_FORMAT_BAYER_BG10         = (GX_PIXEL_MONO  | GX_PIXEL_16BIT | 0x000F),//0x110000F,
	GX_PIXEL_FORMAT_BAYER_GR12         = (GX_PIXEL_MONO  | GX_PIXEL_16BIT | 0x0010),//0x1100010,              
	GX_PIXEL_FORMAT_BAYER_RG12         = (GX_PIXEL_MONO  | GX_PIXEL_16BIT | 0x0011),//0x1100011,
	GX_PIXEL_FORMAT_BAYER_GB12         = (GX_PIXEL_MONO  | GX_PIXEL_16BIT | 0x0012),//0x1100012,
	GX_PIXEL_FORMAT_BAYER_BG12         = (GX_PIXEL_MONO  | GX_PIXEL_16BIT | 0x0013),//0x1100013,
	GX_PIXEL_FORMAT_BAYER_GR10_PACKED  = (GX_PIXEL_MONO  | GX_PIXEL_12BIT | 0x0026),//0x10C0026,                
	GX_PIXEL_FORMAT_BAYER_RG10_PACKED  = (GX_PIXEL_MONO  | GX_PIXEL_12BIT | 0x0027),//0x10C0027,
	GX_PIXEL_FORMAT_BAYER_GB10_PACKED  = (GX_PIXEL_MONO  | GX_PIXEL_12BIT | 0x0028),//0x10C0028,
	GX_PIXEL_FORMAT_BAYER_BG10_PACKED  = (GX_PIXEL_MONO  | GX_PIXEL_12BIT | 0x0029),//0x10C0029,
	GX_PIXEL_FORMAT_BAYER_GR12_PACKED  = (GX_PIXEL_MONO  | GX_PIXEL_12BIT | 0x002A),//0x10C002A,              
	GX_PIXEL_FORMAT_BAYER_RG12_PACKED  = (GX_PIXEL_MONO  | GX_PIXEL_12BIT | 0x002B),//0x10C002B,
	GX_PIXEL_FORMAT_BAYER_GB12_PACKED  = (GX_PIXEL_MONO  | GX_PIXEL_12BIT | 0x002C),//0x10C002C,
	GX_PIXEL_FORMAT_BAYER_BG12_PACKED  = (GX_PIXEL_MONO  | GX_PIXEL_12BIT | 0x002D),//0x10C002D,
	GX_PIXEL_FORMAT_BAYER_GR16         = (GX_PIXEL_MONO  | GX_PIXEL_16BIT | 0x002E),//0x110002E,                
	GX_PIXEL_FORMAT_BAYER_RG16         = (GX_PIXEL_MONO  | GX_PIXEL_16BIT | 0x002F),//0x110002F,
	GX_PIXEL_FORMAT_BAYER_GB16         = (GX_PIXEL_MONO  | GX_PIXEL_16BIT | 0x0030),//0x1100030,
	GX_PIXEL_FORMAT_BAYER_BG16         = (GX_PIXEL_MONO  | GX_PIXEL_16BIT | 0x0031),//0x1100031,
	GX_PIXEL_FORMAT_RGB8_PACKED        = (GX_PIXEL_COLOR | GX_PIXEL_24BIT | 0x0014),//0x2180014,
	GX_PIXEL_FORMAT_BGR8_PACKED        = (GX_PIXEL_COLOR | GX_PIXEL_24BIT | 0x0015),//0x2180015,
	GX_PIXEL_FORMAT_RGBA8_PACKED       = (GX_PIXEL_COLOR | GX_PIXEL_32BIT | 0x0016),//0x2200016,
	GX_PIXEL_FORMAT_BGRA8_PACKED       = (GX_PIXEL_COLOR | GX_PIXEL_32BIT | 0x0017),//0x2200017,
	GX_PIXEL_FORMAT_RGB10_PACKED       = (GX_PIXEL_COLOR | GX_PIXEL_48BIT | 0x0018),//0x2300018,
	GX_PIXEL_FORMAT_BGR10_PACKED       = (GX_PIXEL_COLOR | GX_PIXEL_48BIT | 0x0019),//0x2300019,
	GX_PIXEL_FORMAT_RGB12_PACKED       = (GX_PIXEL_COLOR | GX_PIXEL_48BIT | 0x001A),//0x230001A,
	GX_PIXEL_FORMAT_BGR12_PACKED       = (GX_PIXEL_COLOR | GX_PIXEL_48BIT | 0x001B),//0x230001B,
	GX_PIXEL_FORMAT_RGB16_PACKED       = (GX_PIXEL_COLOR | GX_PIXEL_48BIT | 0x0033),//0x2300033,
	GX_PIXEL_FORMAT_BGR10V1_PACKED     = (GX_PIXEL_COLOR | GX_PIXEL_32BIT | 0x001C),//0x220001C,
	GX_PIXEL_FORMAT_BGR10V2_PACKED     = (GX_PIXEL_COLOR | GX_PIXEL_32BIT | 0x001D),//0x220001D,
	GX_PIXEL_FORMAT_RGB12V1_PACKED     = (GX_PIXEL_COLOR | GX_PIXEL_36BIT | 0x0034),//0x2240034,
	GX_PIXEL_FORMAT_YUV411_PACKED      = (GX_PIXEL_COLOR | GX_PIXEL_12BIT | 0x001E),//0x20C001E,
	GX_PIXEL_FORMAT_YUV422_PACKED      = (GX_PIXEL_COLOR | GX_PIXEL_16BIT | 0x001F),//0x210001F,
	GX_PIXEL_FORMAT_YUV422YUYV_PACKED  = (GX_PIXEL_COLOR | GX_PIXEL_16BIT | 0x0032),//0x2100032,
	GX_PIXEL_FORMAT_YUV444_PACKED      = (GX_PIXEL_COLOR | GX_PIXEL_24BIT | 0x0020),//0x2180020,
	GX_PIXEL_FORMAT_RGB8_PLANAR        = (GX_PIXEL_COLOR | GX_PIXEL_24BIT | 0x0021),//0x2180021,
	GX_PIXEL_FORMAT_RGB10_PLANAR       = (GX_PIXEL_COLOR | GX_PIXEL_48BIT | 0x0022),//0x2300022,
	GX_PIXEL_FORMAT_RGB12_PLANAR       = (GX_PIXEL_COLOR | GX_PIXEL_48BIT | 0x0023),//0x2300023,
	GX_PIXEL_FORMAT_RGB16_PLANAR       = (GX_PIXEL_COLOR | GX_PIXEL_48BIT | 0x0024),//0x2300024,
}GX_PIXEL_FORMAT_ENTRY;

typedef enum GX_ACQUISITION_MODE_ENTRY
{
	GX_ACQ_MODE_SINGLE_FRAME = 0,                          ///<单帧模式
	GX_ACQ_MODE_MULITI_FRAME = 1,                          ///<多帧模式
	GX_ACQ_MODE_CONTINUOUS   = 2,                          ///<连续模式
}GX_ACQUISITION_MODE_ENTRY;

typedef enum GX_TRIGGER_MODE_ENTRY
{
	GX_TRIGGER_MODE_OFF = 0,                             ///< 关闭触发模式
	GX_TRIGGER_MODE_ON  = 1,                             ///< 打开触发模式
}GX_TRIGGER_MODE_ENTRY;

typedef enum GX_TRIGGER_ACTIVATION_ENTRY
{
	GX_TRIGGER_ACTIVATION_FALLINGEDGE = 0,               ///< 下降沿触发
	GX_TRIGGER_ACTIVATION_RISINGEDGE  = 1,               ///< 上升沿触发
}GX_TRIGGER_ACTIVATION_ENTRY;

typedef enum GX_TRIGGER_SWITCH_ENTRY
{
	GX_TRIGGER_SWITCH_OFF = 0,                           ///< 关闭外触发
	GX_TRIGGER_SWITCH_ON  = 1,                           ///< 打开外触发
}GX_TRIGGER_SWITCH_ENTRY;

typedef enum GX_EXPOSURE_AUTO_ENTRY
{
	GX_EXPOSURE_AUTO_OFF        = 0,                     ///< 关闭自动曝光
	GX_EXPOSURE_AUTO_CONTINUOUS = 1,                     ///< 连续自动曝光
	GX_EXPOSURE_AUTO_ONCE       = 2,                     ///< 单次自动曝光
}GX_EXPOSURE_AUTO_ENTRY;

typedef enum GX_USER_OUTPUT_SELECTOR_ENTRY
{
	GX_USER_OUTPUT_SELECTOR_OUTPUT0 = 1,                   ///<输出0
	GX_USER_OUTPUT_SELECTOR_OUTPUT1 = 2,                   ///<输出1
	GX_USER_OUTPUT_SELECTOR_OUTPUT2 = 4,                   ///<输出2
}GX_USER_OUTPUT_SELECTOR_ENTRY;

typedef enum GX_USER_OUTPUT_MODE_ENTRY
{
	GX_USER_OUTPUT_MODE_STROBE      = 0,                   ///<闪光灯
	GX_USER_OUTPUT_MODE_USERDEFINED = 1,                   ///<用户自定义
}GX_USER_OUTPUT_MODE_ENTRY;

typedef enum GX_STROBE_SWITCH_ENTRY
{
	GX_STROBE_SWITCH_OFF = 0,                            ///< 关闭闪光灯开关
	GX_STROBE_SWITCH_ON  = 1,                            ///< 打开闪光灯开关
}GX_STROBE_SWITCH_ENTRY;

typedef enum GX_GAIN_AUTO_ENTRY
{
	GX_GAIN_AUTO_OFF        = 0,                         ///< 关闭自动增益
	GX_GAIN_AUTO_CONTINUOUS = 1,                         ///< 连续自动增益
	GX_GAIN_AUTO_ONCE       = 2,                         ///< 单次自动增益
}GX_GAIN_AUTO_ENTRY;

typedef enum GX_GAIN_SELECTOR_ENTRY
{
	GX_GAIN_SELECTOR_ALL   = 0,                          ///< 所有增益通道
	GX_GAIN_SELECTOR_RED   = 1,                          ///< 红通道增益
	GX_GAIN_SELECTOR_GREEN = 2,                          ///< 绿通道增益
	GX_GAIN_SELECTOR_BLUE  = 3,                          ///< 蓝通道增益
}GX_GAIN_SELECTOR_ENTRY;

typedef enum GX_BLACKLEVEL_AUTO_ENTRY
{
	GX_BLACKLEVEL_AUTO_OFF        = 0,                   ///< 关闭自动黑电平
	GX_BLACKLEVEL_AUTO_CONTINUOUS = 1,                   ///< 连续自动黑电平
	GX_BLACKLEVEL_AUTO_ONCE       = 2,                   ///< 单次自动黑电平
}GX_BLACKLEVEL_AUTO_ENTRY;

typedef enum GX_BLACKLEVEL_SELECTOR_ENTRY
{
	GX_BLACKLEVEL_SELECTOR_ALL   = 0,                    ///< 所有黑电平通道
	GX_BLACKLEVEL_SELECTOR_RED   = 1,                    ///< 红通道黑电平
	GX_BLACKLEVEL_SELECTOR_GREEN = 2,                    ///< 绿通道黑电平
	GX_BLACKLEVEL_SELECTOR_BLUE  = 3,                    ///< 蓝通道黑电平
}GX_BLACKLEVEL_SELECTOR_ENTRY;

typedef enum GX_BALANCE_WHITE_AUTO_ENTRY
{
	GX_BALANCE_WHITE_AUTO_OFF        = 0,                ///< 关闭自动白平衡
	GX_BALANCE_WHITE_AUTO_CONTINUOUS = 1,                ///< 连续自动白平衡
	GX_BALANCE_WHITE_AUTO_ONCE       = 2,                ///< 单次自动白平衡
}GX_BALANCE_WHITE_AUTO_ENTRY;

typedef enum GX_BALANCE_RATIO_SELECTOR_ENTRY
{
	GX_BALANCE_RATIO_SELECTOR_RED   = 0,                   ///<红通道
	GX_BALANCE_RATIO_SELECTOR_GREEN = 1,                   ///<绿通道
	GX_BALANCE_RATIO_SELECTOR_BLUE  = 2,                   ///<蓝通道
}GX_BALANCE_RATIO_SELECTOR_ENTRY;

typedef enum GX_COLOR_CORRECT_ENTRY
{
	GX_COLOR_CORRECT_OFF = 0,                            ///< 关闭自动颜色校正
	GX_COLOR_CORRECT_ON  = 1,                            ///< 打开自动颜色校正
}GX_COLOR_CORRECT_ENTRY;

typedef enum GX_DEAD_PIXEL_CORRECT_ENTRY
{
	GX_DEAD_PIXEL_CORRECT_OFF = 0,                       ///< 关闭自动坏点校正
	GX_DEAD_PIXEL_CORRECT_ON  = 1,                       ///< 打开自动坏点校正
}GX_DEAD_PIXEL_CORRECT_ENTRY;

typedef enum GX_AA_LIGHT_ENVIRMENT_ENTRY
{
	GX_AA_LIGHT_ENVIRMENT_NATURELIGHT = 0,                 ///<自然光
	GX_AA_LIGHT_ENVIRMENT_AC50HZ      = 1,                 ///<50赫兹日光灯
	GX_AA_LIGHT_ENVIRMENT_AC60HZ      = 2,                 ///<60赫兹日光灯
}GX_AALIGHT_ENVIRMENT_ENTRY;

typedef enum GX_USER_SET_SELECTOR_ENTRY
{
	GX_ENUM_USER_SET_SELECTOR_DEFAULT  = 0,                 ///<默认参数组
	GX_ENUM_USER_SET_SELECTOR_USERSET1 = 1,                 ///<用户参数组1
	GX_ENUM_USER_SET_SELECTOR_USERSET2 = 2,                 ///<用户参数组2
}GX_USER_SET_SELECTOR_ENTRY;

//------------------------------------------------------------------------------
//  结构体类型定义
//------------------------------------------------------------------------------

#define GX_INFO_LENGTH_8_BYTE   (8)  ///< 8字节
#define GX_INFO_LENGTH_32_BYTE  (32) ///< 32字节
#define GX_INFO_LENGTH_64_BYTE  (64) ///< 64字节
#define GX_INFO_LENGTH_128_BYTE (128)///< 128字节

typedef struct GX_DEVICE_BASE_INFO 
{
	char szVendorName[GX_INFO_LENGTH_32_BYTE];              ///< 厂商名称
	char szModelName[GX_INFO_LENGTH_32_BYTE];               ///< 设备类型名称
	char szSN[GX_INFO_LENGTH_32_BYTE];                      ///< 设备序列号
	char szDisplayName[GX_INFO_LENGTH_128_BYTE + 4];        ///< 设备展示名称
	char szDeviceID[GX_INFO_LENGTH_64_BYTE + 4];            ///< 设备唯一标识
	char szUserID[GX_INFO_LENGTH_64_BYTE + 4];              ///< 用户自定义名称
	GX_ACCESS_STATUS_CMD  accessStatus;                     ///< 设备当前支持的访问状态
	GX_DEVICE_CLASS   deviceClass;                          ///< 设备种类，比如USB2.0、GEV	
	char reserved[300];                                     ///< 保留
}GX_DEVICE_BASE_INFO;

typedef struct GX_DEVICE_IP_INFO 
{
	char szMAC[32];                 ///< MAC地址
	char szIP[32];                  ///< IP地址
	char szSubNetMask[32];          ///< 子网掩码
	char szGateWay[32];             ///< 网关	
	char reserved[512];             ///< 保留
}GX_DEVICE_IP_INFO;

typedef struct GX_OPEN_PARAM 
{
	char           *pszContent;        ///< 输入参数内容
	GX_OPEN_MODE   openMode;           ///< 打开方式
	GX_ACCESS_MODE accessMode;         ///< 访问模式
}GX_OPEN_PARAM;

typedef struct GX_FRAME_CALLBACK_PARAM
{
	void*         pUserParam;          ///< 用户私有数据
	int32_t       status;              ///< 获取该帧图像的返回状态
	const  void*  pImgBuf;             ///< 返回的图像buffer地址
	int32_t       nImgSize;            ///< 返回的图像大小
	int32_t       reserved[8];         ///< 保留
}GX_FRAME_CALLBACK_PARAM;

typedef struct GX_UTI_CALLBACK_PARAM
{
	void*               pUserParam;    ///< 用户自定义参数
	GX_UTI_CALLBACK_ID  callbackID;    ///< 回调函数类型
	void*               pOutParam;     ///< 回调函数输出参数,根据回调类型不同而不同
	int32_t             reserved[8];   ///< 保留
}GX_UTI_CALLBACK_PARAM;

typedef struct GX_FRAME_DATA
{
	GX_FRAME_STATUS  nStatus;          ///< 获取该帧的返回状态
	void*         pImgBuf;             ///< 返回的图像buffer
	int32_t       nWidth;              ///< 图像的宽
	int32_t       nHeight;             ///< 图像的高
	int32_t       nPixelFormat;        ///< 图像的PixFormat
	int32_t       nImgSize;            ///< 图像大小数据大小，单位字节（开启chunkdata后，nImgsize为图像数据大小+帧信息大小）
	uint64_t      nFrameID;            ///< 图像的帧号
	uint64_t      nTimestamp;          ///< 图像的时间戳
	int32_t       reserved[3];         ///< 保留
}GX_FRAME_DATA;

typedef struct GX_INT_RANGE
{
	int64_t nMin;                      ///< 整型值最小值
	int64_t nMax;                      ///< 整型值最大值
	int64_t nInc;                      ///< 整型值步长
	int32_t reserved[8];               ///< 保留
}GX_INT_RANGE;

typedef struct GX_FLOAT_RANGE
{
	double  dMin;                       ///< 浮点型最小值
	double  dMax;                       ///< 浮点型最大值
	double  dInc;                       ///< 浮点型步长
	char    szUnit[8];                  ///< 浮点型单位
	int32_t reserved[8];                ///< 保留
}GX_FLOAT_RANGE;

typedef struct GX_ENUM_DESCRIPTION
{
	int64_t nValue;                    ///< 枚举值
	char    szSymbolic[64];            ///< 字符描述
	int32_t reserved[8];               ///< 保留
}GX_ENUM_DESCRIPTION;
//------------------------------------------------------------------------------
//  回调函数类型定义
//------------------------------------------------------------------------------
typedef void (GX_STDC* GXCaptureCallBack) (GX_FRAME_CALLBACK_PARAM *pFrameData); ///< 采集回调函数格式定义
typedef void (GX_STDC* GXUtiCallBack)     (GX_UTI_CALLBACK_PARAM *pUtiData);     ///< 辅助回调函数格式定义

//------------------------------------------------------------------------------
//  标准C API功能函数定义
//------------------------------------------------------------------------------
#define GX_API GX_EXTC GX_STATUS GX_STDC

GX_API GXInitLib();
GX_API GXCloseLib();

GX_API GXUpdateDeviceList         (uint32_t* punNumDevices, int32_t unTimeOut);
GX_API GXGetAllDeviceBaseInfo     (GX_DEVICE_BASE_INFO* pDeviceInfo, size_t* pBufferSize);
GX_API GXGetAllDeviceIPInfo       (GX_DEVICE_IP_INFO* pDeviceInfo, size_t* pBufferSize);
GX_API GXSetSingleDeviceIPInfo    (GX_DEVICE_IP_INFO* pDeviceInfo);
GX_API GXGetLastError             (GX_STATUS *pErrorCode, char *pszErrText, size_t *pSize);

GX_API GXOpenDeviceByIndex        (uint32_t nDeviceIndex, GX_DEV_HANDLE* phDevice);
GX_API GXOpenDevice               (GX_OPEN_PARAM* pOpenParam, GX_DEV_HANDLE* phDevice);
GX_API GXCloseDevice              (GX_DEV_HANDLE hDevice);

GX_API GXIsImplemented		      (GX_DEV_HANDLE hDevice, GX_FEATURE_ID featureID, bool* pbIsImplemented);
GX_API GXIsReadable               (GX_DEV_HANDLE hDevice, GX_FEATURE_ID featureID, bool* pbIsReadable);
GX_API GXIsWritable               (GX_DEV_HANDLE hDevice, GX_FEATURE_ID featureID, bool* pbIsWritable);
GX_API GXGetIntRange		      (GX_DEV_HANDLE hDevice, GX_FEATURE_ID featureID, GX_INT_RANGE* pIntRange);
GX_API GXGetInt				      (GX_DEV_HANDLE hDevice, GX_FEATURE_ID featureID, int64_t* pnValue);
GX_API GXSetInt				      (GX_DEV_HANDLE hDevice, GX_FEATURE_ID featureID, int64_t nValue);
GX_API GXGetFloatRange		      (GX_DEV_HANDLE hDevice, GX_FEATURE_ID featureID, GX_FLOAT_RANGE* pFloatRange);
GX_API GXSetFloat                 (GX_DEV_HANDLE hDevice, GX_FEATURE_ID featureID, double dValue);
GX_API GXGetFloat                 (GX_DEV_HANDLE hDevice, GX_FEATURE_ID featureID, double* pdValue);
GX_API GXGetEnumEntryNums         (GX_DEV_HANDLE hDevice, GX_FEATURE_ID featureID, uint32_t* pnEntryNums);
GX_API GXGetEnumDescription       (GX_DEV_HANDLE hDevice, GX_FEATURE_ID featureID, GX_ENUM_DESCRIPTION* pEnumDescription, size_t* pBufferSize);
GX_API GXGetEnum			      (GX_DEV_HANDLE hDevice, GX_FEATURE_ID featureID, int64_t* pnValue);
GX_API GXSetEnum			      (GX_DEV_HANDLE hDevice, GX_FEATURE_ID featureID, int64_t nValue);
GX_API GXGetBool			      (GX_DEV_HANDLE hDevice, GX_FEATURE_ID featureID, bool* pbValue);
GX_API GXSetBool			      (GX_DEV_HANDLE hDevice, GX_FEATURE_ID featureID, bool bValue);
GX_API GXGetStringLength	      (GX_DEV_HANDLE hDevice, GX_FEATURE_ID featureID, size_t* pnSize);
GX_API GXGetString			      (GX_DEV_HANDLE hDevice, GX_FEATURE_ID featureID, char* pszContent, size_t* pnSize);
GX_API GXSetString			      (GX_DEV_HANDLE hDevice, GX_FEATURE_ID featureID, char* pszContent);
GX_API GXGetBufferLength	      (GX_DEV_HANDLE hDevice, GX_FEATURE_ID featureID, size_t* pnSize);
GX_API GXGetBuffer			      (GX_DEV_HANDLE hDevice, GX_FEATURE_ID featureID, uint8_t* pBuffer, size_t* pnSize);
GX_API GXSetBuffer			      (GX_DEV_HANDLE hDevice, GX_FEATURE_ID featureID, uint8_t* pBuffer, size_t nSize);
GX_API GXSendCommand		      (GX_DEV_HANDLE hDevice, GX_FEATURE_ID featureID);
GX_API GXGetFeatureName           (GX_DEV_HANDLE hDevice, GX_FEATURE_ID featureID, char* pszName, size_t* pnSize); 

//说明以下四个接口在该头文件版本中暂不支持
//GX_API GXRegisterCaptureCallback  (GX_DEV_HANDLE hDevice, void *pFrameData, GXCaptureCallBack callBackFun);
//GX_API GXUnregisterCaptureCallback(GX_DEV_HANDLE hDevice);
//GX_API GXRegisterUtiCallback      (GX_DEV_HANDLE hDevice, GX_UTI_CALLBACK_ID callbackID, void *pUtiParam, GXUtiCallBack callbackFun);
//GX_API GXUnregisterUtiCallback    (GX_DEV_HANDLE hDevice, GX_UTI_CALLBACK_ID callbackID);
GX_API GXGetImage                 (GX_DEV_HANDLE hDevice, GX_FRAME_DATA *pFrameData, int32_t nTimeout);

#endif  //GX_GALAXY_H
