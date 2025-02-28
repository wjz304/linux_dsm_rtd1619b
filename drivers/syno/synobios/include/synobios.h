#ifndef MY_ABC_HERE
#define MY_ABC_HERE
#endif
// Copyright (c) 2000-2022 Synology Inc. All rights reserved.
#ifndef __SYNOBIOS_OEM_H_
#define __SYNOBIOS_OEM_H_

#include <linux/synobios.h>

#ifndef SYNO_HI_3535
#include <linux/syno.h>
#endif

#include "syno_ttyS.h"

#if defined(MY_DEF_HERE) || defined(CONFIG_SYNO_HW_VERSION)
#include <linux/version.h>
#include <linux/string.h>
extern char gszSynoHWVersion[];
#endif /* MY_DEF_HERE || CONFIG_SYNO_HW_VERSION */

#ifndef BCD_TO_BIN
#define BCD_TO_BIN(val) ((val)=((val)&15) + ((val)>>4)*10)
#endif
#ifndef BIN_TO_BCD
#define BIN_TO_BCD(val) ((val)=(((val)/10)<<4) + (val)%10)
#endif
#ifndef BCD2BIN
#define BCD2BIN(val) (((val)&15) + ((val)>>4)*10)
#endif
#ifndef BIN2BCD
#define BIN2BCD(val) ((((val)/10)<<4) + (val)%10)
#endif

#define SYNOBIOS_MAJOR    201
#define SYNOBIOS_NEVENTS  128

int GetHWMONSupport(SYNO_HWMON_SUPPORT *hwmon_support);

/*
 * SynoBios Hardware status structure is as following
 * ThermalStage:	0~3: Normal, Warm, Hot, Critical, four stages
 * MemEccCount:		0~X: the count of Memory ECC
 * ButtonStatus:	each bit presents a button status pressed(1) or not(0)
 * HardDiskStatus:	each bit presents a HD status removed(1) or not(0)
 * AcPowerStatus:	a boolean value, 0:AC Power failed; 1:AC Power on-line
 */
/*#define LCD_STR_LEN		40
typedef struct _Synohw {
	unsigned long	AcPowerStatus;
	unsigned long	ThermalStage;
	unsigned long	MemEccCount;
	unsigned long	ButtonStatus;
	unsigned long	HardDiskStatus;
	unsigned long	FanFaultStatus;
	unsigned long	PowerSupplyFault;
	unsigned long	PowerSupplyPresent;
	unsigned long	BatteryStatus;
	unsigned long	BatteryFault;
	char	 	szLcdStr[LCD_STR_LEN];
} SYNOHW;

typedef struct _Synohw_custom {
	unsigned long	obj_1;
	unsigned long	obj_2;
	unsigned long	obj_3;
	unsigned long	obj_4;
	unsigned long	obj_5;
	unsigned long	obj_6;
	unsigned long	obj_7;
	unsigned long	obj_8;
	unsigned long	obj_9;
	unsigned long	obj_10;
	unsigned long	obj_11;
	unsigned long	obj_12;
	unsigned long	obj_13;
	unsigned long	obj_14;
	unsigned long	obj_15;
	unsigned long	obj_16;
	unsigned long	obj_17;
	unsigned long	obj_18;
	unsigned long	obj_19;
	unsigned long	obj_20;
	unsigned long	obj_21;
	unsigned long	obj_22;
	unsigned long	obj_23;
	unsigned long	obj_24;
	unsigned long	obj_25;
	unsigned long	obj_26;
	unsigned long	obj_27;
	unsigned long	obj_28;
	unsigned long	obj_29;
	unsigned long	obj_30;
	unsigned long	obj_31;
	unsigned long	obj_32;
} SYNOHW_CUSTOM;
*/

extern struct proc_dir_entry *proc_synobios_root;

typedef struct _SynoMsgPkt {
	long	usNum;
	long	usLen;
	char	szMsg[128];
} SYNOMSGPKT;

#define BIOS_STR_LEN_MAX	32

/*typedef struct __bvs {
	int	addr;
	char	szBStr[BIOS_STR_LEN_MAX];
} BVS;

#define valueLongPointer(a) (*((unsigned long *) a))
#define FIELD_SIZE	4
#define LEN_SIZE	4

#define APM_SUCCESS	0
#define APM_FAILURE	1
#define APM_NOT_PRESENT -1

#define USE_BY_CONSOLE	1
#define USE_BY_UPS	0
*/

/*
 * SYNOBIOS Events (status change of SynoBIOS)
 */
#define SYNO_EVENT_CPU_THERMAL_NORMAL   0x0100
#define SYNO_EVENT_CPU_THERMAL_WARM     0x0101
#define SYNO_EVENT_CPU_THERMAL_HEAT     0x0102
#define SYNO_EVENT_CPU_THERMAL_CRITICAL 0x0103

#define SYNO_EVENT_BUTTON_0                  0x0200
#define SYNO_EVENT_BUTTON_MANUTIL            0x0200
#define SYNO_EVENT_BUTTON_NORMAL             0x0201

#define SYNO_EVENT_BUTTON_1                  0x0210
#define SYNO_EVENT_BUTTON_2                  0x0220

#define SYNO_EVENT_BUTTON_SHUTDOWN_PUSHED    0x0221
#define SYNO_EVENT_BUTTON_SHUTDOWN_RELEASED  0x0222
#define SYNO_EVENT_BUTTON_SHUTDOWN           0x0223
#define SYNO_EVENT_BUTTON_BUZZER_CLEAR       0x0224

#define SYNO_EVENT_BUTTON_3                  0x0230
#define SYNO_EVENT_BUTTON_RESET              0x0230

#define SYNO_EVENT_BUTTON_4                  0x0240
#define SYNO_EVENT_USBCOPY_START	     0x0240

#define SYNO_EVENT_BUTTON_5                  0x0250
#define SYNO_EVENT_BUTTON_6                  0x0260
#define SYNO_EVENT_BUTTON_7                  0x0270

#define SYNO_EVENT_RM_HD0               0x0300
#define SYNO_EVENT_PLUG_HD0             0x0400
#define SYNO_EVENT_MEM_ECC_1            0x0501
#define SYNO_EVENT_MEM_ECC_2_OR_MORE    0x0502

#define SYNO_EVENT_ACPOWER_FAIL         0x0600
/*#define SYNO_EVENT_IOERR_HD0            0x0700*/

#define	SYNO_EVENT_FAN_FAIL             0x0900
#define SYNO_EVENT_POWERSUPPLY_FAIL     0x0a00
#define SYNO_EVENT_RM_BATTERY           0x0b00
#define SYNO_EVENT_BATTERY_FAULT        0x0c00

#define SYNO_EVENT_FAN_RECOVER          0x0d00
#define SYNO_EVENT_BATTERY_RECOVER      0x0e00
#define SYNO_EVENT_INSERT_BATTERY       0x0f00
#define SYNO_EVENT_POWERSUPPLY_RECOVER  0x1000

#define SYNO_EVENT_NETCARD_ATTACH       0x1100
#define SYNO_EVENT_NETCARD_DETACH       0x1200

#define SYNO_EVENT_RM_SCSI0             0x1300
#define SYNO_EVENT_PLUG_SCSI0           0x1400

#define SYNO_EVENT_HDFAIL_HD0           0x1500
#define SYNO_EVENT_HDRESUME_HD0         0x1600

#define SYNO_EVENT_IOERR_HD0            0x1700

#define SYNO_EVENT_DETECT_CARD_CHANGE   0x1800

#define SYNO_EVENT_RAID                 0x1900

#if defined(MY_DEF_HERE) || defined(CONFIG_SYNO_MD_SYNC_STATUS_REPORT)
#define SYNO_EVENT_RAID_SYNC            0x1a00
#endif /* defined(MY_DEF_HERE) || defined(CONFIG_SYNO_MD_SYNC_STATUS_REPORT) */


#define SYNO_EVENT_SHUTDOWN_LOG			0x2200
#define SYNO_EVENT_EBOX_REFRESH      0x2300

#if defined(MY_DEF_HERE) || defined(CONFIG_SYNO_ECC_NOTIFICATION)
#define SYNO_EVENT_ECC_NOTIFICATION		0x2400
#endif /* MY_DEF_HERE || CONFIG_SYNO_ECC_NOTIFICATION */

#if defined(MY_DEF_HERE) || defined(CONFIG_SYNO_MD_AUTO_REMAP_REPORT)
#define SYNO_EVENT_RAID_REMAP_RECORD 0x2500
#endif /* defined(MY_DEF_HERE) || defined(CONFIG_SYNO_MD_AUTO_REMAP_REPORT) */


#if defined(MY_DEF_HERE) || defined(CONFIG_SYNO_EXT4_ERROR_REPORT)
#define SYNO_EVENT_ERROR_FS 0x2900
#endif /* MY_DEF_HERE || CONFIG_SYNO_EXT4_ERROR_REPORT */




#if defined(MY_DEF_HERE) || defined(CONFIG_SYNO_BTRFS_DATA_CORRECTION)
#define SYNO_EVENT_ERROR_FS_BTRFS 0x2d00
#define SYNO_EVENT_ERROR_BTRFS_META_CORRUPTED 0x2d01
#endif /* MY_DEF_HERE || CONFIG_SYNO_BTRFS_DATA_CORRECTION */

#if defined(MY_DEF_HERE) || defined(CONFIG_SYNO_OOM_NOTIFICATION)
#define SYNO_EVENT_ERROR_OOM 0x2f00
#endif /* (MY_DEF_HERE) || (CONFIG_SYNO_OOM_NOTIFICATION) */

#ifdef CONFIG_SYNO_SYNOBIOS_EVENT
/* SYNO_SYNOBIOS_EVENT moves event define to linux_kernel_version/include/uapi/linux/synobios.h */
#else /* CONFIG_SYNO_SYNOBIOS_EVENT */
#define SYNO_EVENT_USB_PROHIBIT           0x1b00
#define SYNO_EVENT_CONSOLE_PROHIBIT       0x1c00
#define SYNO_EVENT_MICROP_GET             0x1d00
#define SYNO_EVENT_DISK_PWR_RESET         0x2700
#define SYNO_EVENT_DISK_PORT_DISABLED     0x2800
#define SYNO_EVENT_SATA_ERROR_REPORT      0x2a00
#define SYNO_EVENT_WAKE_FROM_DEEP_SLEEP   0x2b00
#define SYNO_EVENT_DISK_RETRY_REPORT      0x2c00
#define SYNO_EVENT_DSIK_POWER_SHORT_BREAK 0x2e00
#define SYNO_EVENT_DISK_TIMEOUT_REPORT    0x3000
#define SYNO_EVENT_DISK_RESET_FAIL_REPORT 0x3100
#define SYNO_EVENT_DISK_PORT_LOST         0x3200
#define SYNO_EVENT_SCSI_ERROR             0x3300
#define SYNO_EVENT_EBOX_RESET             0x3400
#endif /* CONFIG_SYNO_SYNOBIOS_EVENT */


#define SYNO_EVENT_BACK_TEMP_CRITICAL   0x4004
#define SYNO_EVENT_BACK_TEMP_HIGH       0x4003
#define SYNO_EVENT_BACK_TEMP_HEAT       0x4002
#define SYNO_EVENT_BACK_TEMP_WARM       0x4001
#define SYNO_EVENT_CASE_TEMP_CRITICAL   0x4104
#define SYNO_EVENT_CASE_TEMP_HIGH       0x4103
#define SYNO_EVENT_CASE_TEMP_HEAT       0x4102
#define SYNO_EVENT_CASE_TEMP_WARM       0x4101
#define SYNO_EVENT_VOLPIN0_ABNORMAL     0x4200
#define SYNO_EVENT_VOLPIN1_ABNORMAL     0x4201
#define SYNO_EVENT_VOLPIN2_ABNORMAL     0x4202
#define SYNO_EVENT_VOLPIN3_ABNORMAL     0x4203
#define SYNO_EVENT_VOLPIN4_ABNORMAL     0x4204
#define SYNO_EVENT_VOLPIN5_ABNORMAL     0x4205
#define SYNO_EVENT_VOLPIN6_ABNORMAL     0x4206
#define SYNO_EVENT_VOLPIN7_ABNORMAL     0x4207
#define SYNO_EVENT_VOLPIN8_ABNORMAL     0x4208
#define SYNO_EVENT_VOLPIN9_ABNORMAL     0x4209

#define SYNO_EVENT_USBSTATION_RESET 	0x5000
#define SYNO_EVENT_USBSTATION_EJECT 	0x5001

#define SYNO_EVENT_WIFIWPS              0x6000
#define DRIVER_CLASS_FXP                0x00
#define DRIVER_CLASS_EM                 0x10
#define DRIVER_CLASS_RL                 0x20


/*************************************************************************
 * Ioctl definitions
 ************************************************************************/
/*MUST mentain the order in RTC structure*/

typedef struct _SynoRtcControlPkt {
	char    ControlR_1;
	char    ControlR_2;
} SYNORTCCONTROLPKT;

typedef struct _SynoRtcTimePkt {
	unsigned char    sec;
	unsigned char    min;
	unsigned char    hour;
	unsigned char    weekday;
	unsigned char    day;
	unsigned char    month;
	// Value starts from 1900 (in dec)
	unsigned char    year;
} SYNORTCTIMEPKT;

typedef struct _tag_SYNO_MEM_ACCESS {
	unsigned int addr;
	unsigned int value;
} SYNO_MEM_ACCESS;

/* for auto poweron functions */
#define I2C_RTC_ADDR            0x32
#define I2C_RTC_TIMER_OFFSET    0x00
#define I2C_RTC_TIME_TRIM_OFFSET 0x07
#define I2C_RTC_ALARMA_OFFSET   0x08
#define I2C_RTC_ALARMB_OFFSET   0x0B
#define I2C_RTC_CONTROL1_OFFSET 0x0E
#define I2C_RTC_CONTROL2_OFFSET 0x0F

#define I2C_RTC_ALARMA_ENABLE   0x80
#define I2C_RTC_ALARMB_ENABLE   0x40
#define I2C_RTC_ALARMAB_SL      0x20

#define I2C_RTC_ALARMA_24HOUR   0x20

#define I2C_TEMPERATURE_ADDR    0x48

typedef enum {
	AUTO_POWERON_SUN = 0x01,
	AUTO_POWERON_MON = 0x02,
	AUTO_POWERON_TUE = 0x04,
	AUTO_POWERON_WED = 0x08,
	AUTO_POWERON_THU = 0x10,
	AUTO_POWERON_FRI = 0x20,
	AUTO_POWERON_SAT = 0x40,
} AUTO_POWERON_WEEKDAY;
#define AUTO_POWERON_WEEKDAY_MASK ( AUTO_POWERON_SUN | \
									AUTO_POWERON_MON | \
									AUTO_POWERON_TUE | \
									AUTO_POWERON_WED | \
									AUTO_POWERON_THU | \
									AUTO_POWERON_FRI | \
									AUTO_POWERON_SAT )

typedef enum {
	SYNO_AUTO_POWERON_DISABLE = 0,
	SYNO_AUTO_POWERON_ENABLE = 1,
} SYNO_AUTO_POWERON_EN;

typedef struct _SynoRtcAlarmPkt {
	unsigned char    min;			// BCD formatted 24-hour: 17 --> 0x23, we can call DEC2BCD() to convert this field.
	unsigned char    hour;			// BCD formatted 24-hour: 17 --> 0x23, we can call DEC2BCD() to convert this field.
	unsigned char    weekdays;		// 7-bit day of week: [SUN][MON][TUE][WED][THU][FRI][SAT]
} SYNORTCALARMPKT;

#define MAX_POWER_SCHEDULE_TASKS 100

typedef struct _SynoAutoPowerOn {
	int num;
	SYNO_AUTO_POWERON_EN enabled;
	SYNORTCALARMPKT RtcAlarmPkt[MAX_POWER_SCHEDULE_TASKS];
} SYNO_AUTO_POWERON;

/*for test only*/
typedef struct _SynoRtcPkt {
	char    rg[16];
} SYNORTCPKT;

typedef struct _SynoPWMCTL {
	int blSetPWM;
	int hz;
	int duty_cycle;
	int rpm;
} SynoPWMCTL;

#define SYNOBIOS_EVENTDATA_NUM_MAX 8
typedef struct _tag_SynobiosEvent {
    unsigned int event;
    unsigned long long data[SYNOBIOS_EVENTDATA_NUM_MAX];
} SYNOBIOSEVENT;

typedef enum {
	DISK_WAKE_UP = 0,
} SYNO_DISK_HIBERNATION_EVENT;

typedef struct _tag_DiskLedStatus {
    int diskno;
    SYNO_DISK_LED status;
    int iNodeNameLen; /* length of szNodeName */
    const char* szNodeName;
} DISKLEDSTATUS;

typedef enum {
	AHA_LED_OFF = 0,
	AHA_LED_GREEN_SOLID,
	AHA_LED_ORANGE_SOLID,
} SYNO_AHA_LED;

typedef	enum {
	FAN_STATUS_UNKNOWN_UART_BUSY = -2,
	FAN_STATUS_UNKNOWN = -1,
	FAN_STATUS_STOP = 0,
	FAN_STATUS_RUNNING = 1,
} FAN_STATUS;

typedef	enum {
    BACKPLANE_STATUS_UNKNOWN = -1,
	BACKPLANE_STATUS_ERROR = 0,
	BACKPLANE_STATUS_NORMAL = 1,
} BACKPLANE_STATUS;

typedef enum {
    TEMPERATURE_OVER_NORMAL = -7,
} SYNO_SHUTDOWN_LOG;

/* When you add a new FAN_SPEED. Please modify the parsing in libsynosdk/external/?match.c */
typedef	enum {
	/* The FAN_SPEED_* is ordered from low to high speed.
 	 * Note, the FAN_SPEED_UNKNOWN = 0, must not move, we will init FAN_SPEED value to FAN_SPEED_UNKNOWN
	 * and will use it to compare with other speeds. So it must at the lowest order */
	FAN_SPEED_UNKNOWN = 0,
	FAN_SPEED_STOP,
	FAN_SPEED_ULTRA_LOW,
	FAN_SPEED_VERY_LOW,
	FAN_SPEED_LOW,
	FAN_SPEED_MIDDLE,
	FAN_SPEED_HIGH,
	FAN_SPEED_VERY_HIGH,
	FAN_SPEED_ULTRA_HIGH,
	FAN_SPEED_FULL,
	/* FAN_SPEED_TEST_X is used inside manutild for fan testing*/
	FAN_SPEED_TEST_0,
	FAN_SPEED_TEST_1,
	FAN_SPEED_TEST_2,
	FAN_SPEED_TEST_3,
	FAN_SPEED_TEST_4,
	FAN_SPEED_TEST_5,
	FAN_SPEED_TEST_6,
	FAN_SPEED_TEST_7,
	FAN_SPEED_TEST_8,
	FAN_SPEED_TEST_9,
	FAN_SPEED_TEST_10,
	FAN_SPEED_TEST_11,
	FAN_SPEED_TEST_12,
	FAN_SPEED_TEST_13,
	FAN_SPEED_TEST_14,
	FAN_SPEED_TEST_15,
	FAN_SPEED_TEST_16,
	FAN_SPEED_TEST_17,
	/* PWM format is only for bandon and QC test */
	FAN_SPEED_PWM_FORMAT_SHIFT = 1000,
} FAN_SPEED;

/* Mapping FAN_SPEED to duty cycle */
typedef struct _tag_PWM_FAN_SPEED_MAPPING_ {
	FAN_SPEED fanSpeed;
	int       iDutyCycle; // Range [0, 99]
} PWM_FAN_SPEED_MAPPING;

typedef struct _tag_FanStatus {
	int        fanno;   // CS406 ==>1, RS406 ==> 1,2,3
	FAN_STATUS status;  // 1 ==> stop, 0 ==> running
	FAN_SPEED  speed;   // 1 ==> high, 0 ==> low
} FANSTATUS;

/* shift 8bit and set hz value in it */
#define FAN_SPEED_SHIFT_SET(duty_cycle, hz)   ((duty_cycle) + ((hz) << 8) + FAN_SPEED_PWM_FORMAT_SHIFT)
#define FAN_SPEED_SHIFT_HZ_GET(speed)		  (((speed) - FAN_SPEED_PWM_FORMAT_SHIFT) >> 8)   /* shift 8bit and get hz value  */
#define FAN_SPEED_SHIFT_DUTY_GET(speed)		  (((speed) - FAN_SPEED_PWM_FORMAT_SHIFT) & 0xFF) /* duty cycle is set in the first 8bit, so get it in first 8bit */

/* if curspeed is stop and nxtspeed is lower than "high", return true. else return false*/
#define IS_FAN_NEED_TO_SPIN_FASTER_FIRST(NxtSpeed, CurSpeed) (((NxtSpeed) < FAN_SPEED_PWM_FORMAT_SHIFT) ? ( (FAN_SPEED_HIGH > (NxtSpeed) && FAN_SPEED_STOP == (CurSpeed) ) || (FAN_SPEED_TEST_5 > (NxtSpeed) && FAN_SPEED_TEST_0 == (CurSpeed) ) ) : (65 > FAN_SPEED_SHIFT_DUTY_GET(NxtSpeed) && (0 == FAN_SPEED_SHIFT_DUTY_GET(CurSpeed) ||  IS_FAN_SET_TO_STOP(CurSpeed))))

#define IS_FAN_SET_TO_STOP(CurSpeed) ((FAN_SPEED_STOP == CurSpeed) || (FAN_SPEED_TEST_0 == CurSpeed))

// Synology Disk Station Brand
enum {
	BRAND_SYNOLOGY		= 0x00,
	BRAND_LOGITEC		= 0x01,
	BRAND_SYNOLOGY_USA	= 0x02,
};

// for hardware capabilities
#define MAX_CAPABILITY 4

typedef enum {
	POWER_STATUS_BAD = 0,
	POWER_STATUS_GOOD,
} SYNO_POWER_STATUS;

// FIXME: only 4 bits, and at most 16 kinds of type fan_t, so please be careful
typedef enum {
	FAN_UNKNOWN,
	FAN_CPLD_SPEED_1LEVEL,
	FAN_CPLD_SPEED_2LEVEL,
	FAN_MULTI_EXCEPT_SLEEP,
	FAN_MICROP_ALERT_FIRM,
	FAN_MICROP_ALERT_SOFT,
	FAN_MICROP_PWM,
	FAN_MULTI_ALWAYS,
	FAN_MICROP_PWM_WITH_CPUFAN,
	FAN_MICROP_PWM_WITH_GPIO,
	FAN_MICROP_PWM_WITH_CPUFAN_AND_GPIO,
	FAN_ADT,
	FAN_ADT_FANFAIL_WITH_MICROP,
	FAN_BMC,
} FAN_T;

typedef enum {
	LED_UNKNOWN,
	LED_DISKS,
	LED_DISK_ESATA,
	LED_DISKS_ALARM,
} LED_T;

#if defined(MY_DEF_HERE) || defined(CONFIG_SYNO_MD_SYNC_STATUS_REPORT)
typedef enum {
	RAID_SYNC_NONE = 0,
	RAID_SYNC_RESYNC,
	RAID_SYNC_REQUESTED_RESYNC,
	RAID_SYNC_CHECK,
	RAID_SYNC_RECOVERY,
	RAID_SYNC_RESHAPE
} RAID_SYNC_TYPE;
#endif /* defined(MY_DEF_HERE) || defined(CONFIG_SYNO_MD_SYNC_STATUS_REPORT) */

typedef enum {
	THERMAL_UNKNOWN,
	THERMAL_YES,
	THERMAL_NO,
} THERMAL_T;

//Support get CPU temperature or not
typedef enum {
	CPUTMP_UNKNOWN,
	CPUTMP_YES,
	CPUTMP_NO,
} CPUTMP_T;

typedef enum {
	DISK_LED_CTRL_UNKNOWN,
	DISK_LED_CTRL_SW,
	DISK_LED_CTRL_HW,
} DISK_LED_CTRL_T;

typedef enum {
	AUTO_POWERON_UNKNOWN,
	AUTO_POWERON_YES,
	AUTO_POWERON_NO,
} AUTO_POWERON_T;

typedef enum {
	POWER_UNKNOWN,
	POWER_SINGLE,
	POWER_DUAL,
} DUAL_POWER_T;

typedef enum {
	USBCOPY_NO = 0,
	USBCOPY_MICROP,
	USBCOPY_GPIO,
} USBCOPY_T;

typedef enum {
	FAN_NUMBER_X = 0,
	FAN_NUMBER_1 = 1,
	FAN_NUMBER_2 = 2,
	FAN_NUMBER_3 = 3,
	FAN_NUMBER_4 = 4,
	FAN_NUMBER_5 = 5,
	FAN_NUMBER_6 = 6,
} FAN_NUMBER_T;

typedef enum _tag_POWER_IN_SEQ {
	POWER_IN_SEQ_OFF,
	POWER_IN_SEQ_2,
	POWER_IN_SEQ_4,
} POWER_IN_SEQ;

typedef enum _tag_RTC_TYPE {
	RTC_UNKNOWN,
	RTC_RICOH,
	RTC_SEIKO,
	RTC_BANDON,
	RTC_MV,
	RTC_DIODES,
} RTC_TYPE;

typedef enum {
	FAN_RPM_RPT_UNKNOWN,
	FAN_RPM_RPT_YES,
	FAN_RPM_RPT_NO,
} FAN_RPM_RPT_T;

typedef enum {
	LCM_UNKNOWN,
	LCM_YES,
	LCM_NO
} LCM_T;

typedef enum {
	WIFI_WPS_NO = 0xFE,
	WIFI_WPS_UNKNOWN = 0xFF
} WIFI_WPS_T;

typedef enum {
	CPU_FREQ_ADJUST_UNKNOWN,
	CPU_FREQ_ADJUST_YES,
	CPU_FREQ_ADJUST_NO,
} CPUFREQ_ADJUST_T;

typedef enum {
	CARDREADER_UNKNOWN,
	CARDREADER_YES,
	CARDREADER_NO,
} CARDREADER_T;

typedef enum {
	MICROP_PWM_UNKNOWN,
	MICROP_PWM_YES,
	MICROP_PWM_NO,
} MICROP_PWM_T;

typedef enum {
	HIBER_LED_UNKNOWN,
	HIBER_LED_STATUS_OFF,    /* status led off, others don't care */
	HIBER_LED_STATUS_BREATH, /* status led breathing */
	HIBER_LED_POWER_ONLY,    /* power led on, others off */
	HIBER_LED_STANDARD_BEHAVIOR,  /* hibernation status off, deep sleep disk off */
} HIBERNATE_LED_T;

//For backward compatibility
#define HIBER_LED_NORMAL HIBER_LED_STATUS_OFF
#define HIBER_LED_ALLOUT HIBER_LED_STATUS_BREATH
#define HIBER_LED_EXCEPTPWR HIBER_LED_POWER_ONLY

typedef enum {
	RTC_NOT_NEED_TO_CORRECT = 0x00,
	RTC_SEIKO_CORR_DEFAULT  = 0x03, /* -5.62 sec/day */
	RTC_SEIKO_CORR_106B1    = 0x57,
	RTC_RICOH_CORR_DEFAULT  = 0x0F,
} RTC_CORRECTION_T;

typedef enum {
	MICROP_ID_710p = 0x31, /* '1' */
	MICROP_ID_411p = 0x33, /* '3' 411+II is the same*/
	MICROP_ID_1010p = 0x32, /* '2' */
	MICROP_ID_1511p = 0x36, /* '6' */
	MICROP_ID_810p = 0x35, /* '5' */
	MICROP_ID_810rp = 0x34, /* '4' */
	MICROP_ID_2211p = 0x37, /* '7' */
	MICROP_ID_2211rp = 0x38, /* '8' */
	MICROP_ID_2411p = 0x39, /* '9' */
	MICROP_ID_3411xs = 0x43, /* 'C' 3412xs is the same */
	MICROP_ID_3411rpxs = 0x41, /* 'A' 3412rpxs is the same */
	MICROP_ID_3611xs = 0x42, /* 'B' 3612xs is the same*/
	MICROP_ID_712p = 0x44, /* 'D' */
	MICROP_ID_412p = 0x45, /* 'E' */
	MICROP_ID_1512p = 0x47, /* 'G' */
	MICROP_ID_1812p = 0x46, /* 'F' */
	MICROP_ID_812p = 0x48, /* 'H' */
	MICROP_ID_812rp = 0x49, /* 'I' */
	MICROP_ID_2212p = 0x4A, /* 'J' */
	MICROP_ID_2212rp = 0x4B, /* 'K' */
	MICROP_ID_2413p = 0x4C, /* 'L' */
	MICROP_ID_10613xsp = 0x4d, /* 'M' */
	MICROP_ID_3413xsp = 0x4e, /* 'N' */
	MICROP_ID_713p = 0x50, /* 'P' */
	MICROP_ID_1513p = 0x51, /* 'Q' */
	MICROP_ID_1813p = 0x52, /* 'R' */
	MICROP_ID_RS2414p = 0x53, /* 'S' */
	MICROP_ID_RS2414rpp = 0x54, /* 'T' */
	MICROP_ID_RS814p = 0x55, /* 'U' */
	MICROP_ID_RS814rpp = 0x56, /* 'V' */
	MICROP_ID_RS3614xsp = 0x57, /* 'W' RS3614xsp */
	MICROP_ID_RS3617xs = 0x57, /* 'W' RS3617xs */
	MICROP_ID_ES3614xsp = 0x57, /* 'W' ES3614xsp */
	MICROP_ID_RS3614xs = 0x5B, /* 'W' RS3614xs */
	MICROP_ID_RS3614rpxs = 0x5C, /* 'W' RS3614rpxs */
	MICROP_ID_RC18015xsp = 0x4d, /* 'M' Temporarily using the same microp ID as 10613 */
	MICROP_ID_RS18016xsp = 0x4d, /* 'M' */
	MICROP_ID_RS18016Dxsp = 0x4d, /* 'M' */
	MICROP_ID_FS3017 = 0x4d, /* 'M' */
	MICROP_ID_DS2414xs = 0x57, /* 'W' DS2414xs */
	MICROP_ID_RS3415xsp = 0x60, /* '`' RS3415xs+ cheese cake*/
	MICROP_ID_3615xs = 0x5D, /* ']' DS3615xs*/
	MICROP_ID_RS3614xsR1 = 0x64, /* 'd' RS3614xs R1 */
	MICROP_ID_RS3614rpxsR1 = 0x65, /* 'e' RS3614rpxs R1 */
	MICROP_ID_UNKNOW = 0xFF,
} SYNO_MICROP_ID;

#define MICROP_LOG_OFF          0
#define MICROP_LOG_ON           1

/* This is for grpup wake setting
 * [7-4]: how many disks are in one group
 * [3-0]: the deno of 7s, it means how
 *        many seconds between each groups
 * ex. 0x47 means 4 disks in one gorup,
 *     and delay 7/7 = 1s between each waking groups
 **/
typedef enum {
	ONE_DISK_DENO_ONE = 0x11,     /* this is default settings */
	FOUR_DISK_DENO_SEVEN = 0x47,  /* if the model is >=8bay, you must use this */
	FIVE_DISK_DENO_ONE = 0x51,    /* for 5bay, wakeup simultaneously, but insert command before wakeup */
	UNKNOW_DISK_DENO = 0x00,
} GROUP_WAKE_CONFIG_T;

typedef enum {
	CPU_UNKNOWN,
	CPU_E5_2620v3,
	CPU_E5_2609v3,
	CPU_E3_1230v2,
	CPU_D_1508,
	CPU_D_1521,
	CPU_D_1527,
	CPU_D_1528,
	CPU_D_1531,
	CPU_D_1541,
	CPU_D_1567,
	CPU_I3_2100,
	CPU_I3_4130,
	CPU_D410,
	CPU_D425,
	CPU_D510,
	CPU_D525,
	CPU_D2700,
	CPU_CE5335,
	CPU_88F6281,
	CPU_88F6282,
	CPU_88F6702,
	CPU_88F6707,
	CPU_88F6720,
	CPU_88F6820,
	CPU_88F6828,
	CPU_88F3710,
	CPU_88F3720,
	CPU_MV78230,
	CPU_8241,
	CPU_8533e,
	CPU_P1022,
	CPU_C2000,
	CPU_LS1024,
	CPU_AL212,
	CPU_AL314,
	CPU_AL514,
	CPU_C2538,
	CPU_BCM58622,
	CPU_J1800,
	CPU_J3355,
	CPU_J3455,
	CPU_J4005,
	CPU_J4025,
	CPU_J4105,
	CPU_J4125,
	CPU_J4205,
	CPU_HI3535,
	CPU_H412,
	CPU_N3000,
	CPU_N3050,
	CPU_N3150,
	CPU_N3700,
	CPU_NVR,
	CPU_N3010,
	CPU_N3060,
	CPU_N3160,
	CPU_N3710,
	CPU_HI3536,
	CPU_RTD1296,
	CPU_RTD1293,
	CPU_C3538,
	CPU_SILVER_4110,
	CPU_RTD1619,
	CPU_D_2143IT,
	CPU_V1500B,
	CPU_SILVER_4210R,
	CPU_SILVER_4214R,
	CPU_V1780B,
	CPU_EPYC7272,
	CPU_R1600,
	CPU_D_5566,
	CPU_RTD1619B,
	CPU_EPYC7232P,
	CPU_5800E,
	CPU_EPYC7443P
} CPU_ARCH_INFO_T;

typedef enum {
	CRYPTO_HW_NONE,
	CRYPTO_A370,
	CRYPTO_A375,
	CRYPTO_A38X,
	CRYPTO_AXP,
	CRYPTO_COMCERTO2K,
	CRYPTO_ALPINE,
	CRYPTO_QORIQ,
	CRYPTO_853X,
	CRYPTO_628X,
	CRYPTO_RTD129X,
	CRYPTO_RTD1619,
	CRYPTO_RTD1619B
} CRYPTO_HW_INFO_T;

/**
 * This structure is used to store types of each module
 * in different DS models, including module fan type,
 * module raid type, ...
 */
typedef struct {
	FAN_T          fan_type          :4;
	LED_T          led_type          :4;
	THERMAL_T      thermal_type      :2;
	DISK_LED_CTRL_T diskled_ctrl_type:2;
	AUTO_POWERON_T auto_poweron_type :2;
	DUAL_POWER_T   dual_power_type   :2;
	USBCOPY_T      usbcopy_type      :2;
	FAN_NUMBER_T   fan_number        :4;
	EUNIT_PWRON_TYPE eunit_pwron_type:4;
	POWER_IN_SEQ   pis_type          :4;
	RTC_TYPE       rtc_type          :4;
	CPUTMP_T       cputmp_type       :2;
	FAN_RPM_RPT_T  fan_rpm_rpt_type  :2;
	LCM_T          lcm_type		 :2;
	WIFI_WPS_T		wifi_wps_type	 :8;
	CPUFREQ_ADJUST_T cpu_freq_adjust :2;
	CARDREADER_T   has_cardreader    :2;
	HIBERNATE_LED_T  hibernate_led   :4;
	RTC_CORRECTION_T rtc_corr_value  :8;
	SYNO_MICROP_ID microp_id         :8;
	GROUP_WAKE_CONFIG_T group_wake_config :8;
	CPU_ARCH_INFO_T cpu_arch_info    :8;
	CRYPTO_HW_INFO_T crypto_hw_info  :8;
} __attribute__((packed)) module_t;

typedef struct _tag_HwCapability {
	char*           szHwVersion;
	CAPABILITY      capability[MAX_CAPABILITY];
} HwCapability;

// Synology Disk Station model
typedef enum {
	MODEL_RS3411rpxs = 0x00,
	MODEL_RS3411xs = 0x01,
	MODEL_RS10613xsp = 0x02,
	MODEL_DS3611xs,
	MODEL_RS3412rpxs,
	MODEL_RS3412xs,
	MODEL_DS3612xs,
	MODEL_RS3413xsp,  
	MODEL_RS3614xs,
	MODEL_RS3614rpxs,
	MODEL_RS3614xsp, 	//10
	MODEL_ES3614xsp,
	MODEL_DS2414xs,
	MODEL_DS412p,
	MODEL_DS713p,
	MODEL_RS812p,
	MODEL_RS812rpp,
	MODEL_DS1812p,
	MODEL_RS2212p,
	MODEL_RS2212rpp,
	MODEL_RS2414p,		//20
	MODEL_RS2414rpp,
	MODEL_DS2413p,
	MODEL_RS812,
	MODEL_DS1512p,
	MODEL_DS112slim,
	MODEL_DS414j,
	MODEL_DS415j,
	MODEL_DS213,
	MODEL_RS214,		//30
	MODEL_DS1513p,
	MODEL_DS1813p,
	MODEL_DS213j,
	MODEL_DS114,
	MODEL_DS214,
	MODEL_DS214p,
	MODEL_DS414,
	MODEL_RS814,
	MODEL_DS114p,
	MODEL_RC18015xsp,  //40
	MODEL_RS814p,
	MODEL_RS814rpp,
	MODEL_DS214se,
	MODEL_DS214play,
	MODEL_DS2415p,
	MODEL_DS414slim,
	MODEL_DS415play,
	MODEL_DS2015xs,
	MODEL_DS115j,
	MODEL_RS3415xsp, 	//50
	MODEL_DS415p,
	MODEL_DS3615xs,
	MODEL_DS1815p,
	MODEL_DS1515p,
	MODEL_DS215j,
	MODEL_RS815p,
	MODEL_RS815rpp,
	MODEL_DS715,
	MODEL_DS115, 		//60
	MODEL_RS815,
	MODEL_RS18016xsp,
	MODEL_VS360hd,
	MODEL_DS1515,
	MODEL_DS215p,
	MODEL_DS416,
	MODEL_RR36015xsppp,
	MODEL_VirtualDSM,
	MODEL_RS18016Dxsp,
	MODEL_RS2416p, 		//70
	MODEL_RS2416rpp,
	MODEL_DS216play,
	MODEL_DS216se,
	MODEL_DS916p,
	MODEL_RSD18016xsp,
	MODEL_DS1616p,
	MODEL_DS716p,
	MODEL_RS3618xs,
	MODEL_RS3617rpxs,
	MODEL_RS3617xsp, 	//80
	MODEL_DS416j,
	MODEL_DS216,
	MODEL_DS416slim,
	MODEL_DS216p,
	MODEL_DS216j,
	MODEL_RS816,
	MODEL_DS116,
	MODEL_DS416play,
	MODEL_DS216pII,
	MODEL_DS716pII, 	//90
	MODEL_RS217,
	MODEL_FS3017,
	MODEL_RS3617xs,
	MODEL_RS2418p,
	MODEL_RS2418rpp,
	MODEL_DS918p,
	MODEL_RS4017xsp,
	MODEL_DS3617xs,
	MODEL_RS18017xsp,
	MODEL_DS218p, 		//100
	MODEL_DS1618p,
	MODEL_C2DSM,
	MODEL_DS1517p,
	MODEL_DS1817p,
	MODEL_DS718p,
	MODEL_FS2017,
	MODEL_DS418j,
	MODEL_DS418,
	MODEL_DS218play,
	MODEL_DS118, 	//110
	MODEL_DS218,
	MODEL_EDS19,
	MODEL_RS819,
	MODEL_DS220j,
	MODEL_DS420j,
	MODEL_DS1817,
	MODEL_DS1517,
	MODEL_DS3017xs,
	MODEL_DS3018xs,
	MODEL_FS1018, 	//120
	MODEL_DS219j,
	MODEL_FS6400,
	MODEL_RS818p,
	MODEL_RS818rpp,
	MODEL_DS418play,
	MODEL_RS2818rpp,
	MODEL_DS219se,
	MODEL_NVR1218,
	MODEL_DS218j,
	MODEL_DS3619xs, //130
	MODEL_DS119j,
	MODEL_RS1619xsp,
	MODEL_TAIPEI,
	MODEL_RS1219p,
	MODEL_DS2419p,
	MODEL_DS419p,
	MODEL_DS1019p,
	MODEL_DS719p,
	MODEL_DS1819p,
	MODEL_DS620slim, //140
	MODEL_RS419p,
	MODEL_DS419slim,
	MODEL_RS1219,
	MODEL_DVA3219,
	MODEL_SA3400,
	MODEL_DS420p,
	MODEL_DS720p,
	MODEL_DS220p,
	MODEL_RS820p,
	MODEL_RS820rpp, //150
	MODEL_FS3400,
	MODEL_SA3600,
	MODEL_FS3600,
	MODEL_HD3400,
	MODEL_DS220play,
	MODEL_DS220,
	MODEL_DS120j,
	MODEL_DS1520p,
	MODEL_RS1220p,
	MODEL_RS1220rpp,
	MODEL_DS1621xsp,
	MODEL_DS920p,
	MODEL_SA6500,
	MODEL_DS1621p,
	MODEL_HD6500,
	MODEL_SA3200d,
	MODEL_DVA3221,
	MODEL_AliDSM,
	MODEL_RS1221p,
	MODEL_RS1221rpp,
	MODEL_DS1821p,
	MODEL_DS2422p,
	MODEL_RS2421p,
	MODEL_RS2421rpp,
	MODEL_RS2821rpp,
	MODEL_FS6600N,
	MODEL_RS3621xsp,
	MODEL_RS3621rpxs,
	MODEL_RS4021xsp,
	MODEL_FS6500,
	MODEL_DS3617xsII,
	MODEL_DS3622xsp,
	MODEL_DS2419pII,
	MODEL_FS2500,
	MODEL_FS2500T,
	MODEL_FS6400N,
	MODEL_DS723p,
	MODEL_DS923p,
	MODEL_RS422p,
	MODEL_DVA1622,
	MODEL_RS4023xsp,
	MODEL_DS223j,
	MODEL_DS1522p,
	MODEL_RS4022xsp,
	MODEL_FS3410,
	MODEL_DS423,
	MODEL_FS6410,
	MODEL_SA6400,
	MODEL_SA6200,
	MODEL_SC6200,
	MODEL_DS223,
	MODEL_RS822p,
	MODEL_RS822rpp,
	MODEL_RS2423p,
	MODEL_RS2423rpp,
	MODEL_DS1823xsp,
	MODEL_RS1623xsp,
	MODEL_DS423p,
	MODEL_SA3410,
	MODEL_SA3610,
	MODEL_DS124,
	MODEL_VS750hd,
	MODEL_SA3400d,
	MODEL_DS224p,
	MODEL_RS4024xsp,
	MODEL_FS6600DN,
	MODEL_DS1623p,
	MODEL_DS1823p,
	MODEL_SC2500,
	MODEL_INVALID
} PRODUCT_MODEL;

typedef struct _tag_SYNO_MODEL_NAME {
	PRODUCT_MODEL	model;
	char*			szHwVersion;
} SYNO_MODEL_NAME;


typedef struct _tag_CPLDReg {
	unsigned char diskledctrl;
	unsigned char diskpowerstate;
	unsigned char modelnumber;
	unsigned char fanstatus;
} CPLDREG;

typedef struct _tag_MEMORY_BYTE {
	unsigned char offset;
	unsigned char value;
} MEMORY_BYTE;

typedef struct _tag_GPIO_PIN {
	int pin;
	int value;
} GPIO_PIN;

typedef struct _tag_POWER_INFO {
	SYNO_POWER_STATUS power_1;
	SYNO_POWER_STATUS power_2;
} POWER_INFO;

typedef enum {
	NET_LINK = 0,
	NET_NOLINK,
} SYNO_NET_LINK_EVENT;


/**
 * from first 0 bit to 6th bit is signature
 * 7th bit indicate component status, true is compoent fail
 * the others bits are id
 */
typedef unsigned int sys_comp_stat_t;
#define COMP_STAT_BITS						sizeof(sys_comp_stat_t)*8

#define SYS_STAT_SIGNATURE_MASK 0x7F
#define SYS_STAT_COMPONENT_MASK 0x80
#define SYS_STAT_ID_MASK ~(0xFF)

#define SYS_STAT_COMPONENT_SHIFT 7
#define SYS_STAT_IDX_SHIFT 8

#define DEFINE_COMP_STAT(x)					sys_comp_stat_t x = 0

#define ID_LIST_GET(sys_comp_stat_t)		(sys_comp_stat_t & SYS_STAT_ID_MASK)
#define ID_LIST_ENCODE(idx)					((1 << idx) << SYS_STAT_IDX_SHIFT)
#define ID_LIST_DECODE(sys_comp_stat_t)		(sys_comp_stat_t >> SYS_STAT_IDX_SHIFT)
#define ID_LIST_EMPTY(sys_comp_stat_t)		(0 == ID_LIST_DECODE(sys_comp_stat_t))
#define SIGNATURE_GET(sig)					(sig & SYS_STAT_SIGNATURE_MASK)
#define SIGNATURE_INCLUDE(sys_comp_stat_t)	(sys_comp_stat_t & SYS_STAT_SIGNATURE_MASK)
#define COMP_STAT_ENCODE(boolean)			(boolean << SYS_STAT_COMPONENT_SHIFT)
#define COMP_FAIL(sys_comp_stat_t)			(sys_comp_stat_t & SYS_STAT_COMPONENT_MASK)

/**
 * This is the primaty key for look up SYNO_SYS_STATUS.
 * When we want add a new component status. You must add a new
 * signature too
 */
typedef enum {
	SIGNATURE_FAN_FAIL = 0x1,
	SIGNATURE_VOLUME_DEGRADE = 0x2,
	SIGNATURE_VOLUME_CRASHED = 0x3,
	SIGNATURE_POWER_FAIL = 0x4,
	SIGNATURE_EBOX_FAN_FAIL = 0x5,
	SIGNATURE_CPU_FAN_FAIL = 0x6,
} SYNO_SYS_STAT_SIGNATURE;

/**
 * These are system components stat, we don't use array here.
 * Because it still need hard code signature in synobios.c
 */
typedef struct _tag_SYNO_SYS_STATUS {
	sys_comp_stat_t fan_fail;
	sys_comp_stat_t volume_degrade;
	sys_comp_stat_t volume_crashed;
	sys_comp_stat_t power_fail;
	sys_comp_stat_t ebox_fan_fail;
	sys_comp_stat_t cpu_fan_fail;
} SYNO_SYS_STATUS;

typedef struct _tag_SYNO_CPU_INFO {
	unsigned int core;
	char clock[16];
#if defined(CONFIG_SYNO_GRANTLEY) || defined(CONFIG_SYNO_PURLEY)
	unsigned int cpucore[CONFIG_SYNO_MULTI_CPU_NUM];
#endif
} SYNO_CPU_INFO;

#define MAX_DISK_PER_INTF 64
#define MAX_DISK_INTF_LEN 32
typedef struct _tag_SYNO_DISK_INTF {
	unsigned char disk_list[MAX_DISK_PER_INTF];
	char intf_name[MAX_DISK_INTF_LEN];
} SYNO_DISK_INTF;
#define MAX_DISK_INTF 2 // for SAS/SATA or U.2 NVMe
typedef struct _tag_SYNO_DISK_INTF_INFO {
	int intf_num;
	SYNO_DISK_INTF disk_intf[MAX_DISK_INTF];
} SYNO_DISK_INTF_INFO;

#define SYNOIO_EXDISPLAY          _IOWR(SYNOBIOS_IOC_MAGIC, 1, struct _SynoMsgPkt)
#define SYNOIO_NEXTEVENT          _IOWR(SYNOBIOS_IOC_MAGIC, 2, u_int)

#define SYNOIO_RTC_CONTROL_READ   _IOWR(SYNOBIOS_IOC_MAGIC, 3, struct _SynoRtcControlPkt)
#define SYNOIO_RTC_CONTROL_WRITE  _IOWR(SYNOBIOS_IOC_MAGIC, 4, struct _SynoRtcControlPkt)
#define SYNOIO_RTC_TIME_READ      _IOWR(SYNOBIOS_IOC_MAGIC, 5, struct _SynoRtcTimePkt)
#define SYNOIO_RTC_TIME_WRITE     _IOWR(SYNOBIOS_IOC_MAGIC, 6, struct _SynoRtcTimePkt)

#define SYNOIO_BUTTON_POWER		_IOWR(SYNOBIOS_IOC_MAGIC, 7, int)
#define SYNOIO_BUTTON_RESET		_IOWR(SYNOBIOS_IOC_MAGIC, 8, int)
#define SYNOIO_BUTTON_USB		_IOWR(SYNOBIOS_IOC_MAGIC, 9, int)

#define SYNOIO_SET_DISK_LED     _IOWR(SYNOBIOS_IOC_MAGIC, 10, DISKLEDSTATUS)
#define SYNOIO_GET_FAN_STATUS   _IOWR(SYNOBIOS_IOC_MAGIC, 11, FANSTATUS)
#define SYNOIO_SET_FAN_STATUS   _IOWR(SYNOBIOS_IOC_MAGIC, 12, FANSTATUS)
#define SYNOIO_GET_DS_MODEL  _IOWR(SYNOBIOS_IOC_MAGIC, 13, int)
#define SYNOIO_GET_DS_BRAND  _IOWR(SYNOBIOS_IOC_MAGIC, 14, int)
#define SYNOIO_GET_CPLD_VERSION _IOWR(SYNOBIOS_IOC_MAGIC, 15, int)
#define SYNOIO_GET_TEMPERATURE  _IOWR(SYNOBIOS_IOC_MAGIC, 16, int)
#define SYNOIO_GET_CPLD_REG  _IOWR(SYNOBIOS_IOC_MAGIC, 17, CPLDREG)
#define SYNOIO_GET_AUTO_POWERON     _IOWR(SYNOBIOS_IOC_MAGIC, 18, SYNO_AUTO_POWERON)
#define SYNOIO_SET_AUTO_POWERON     _IOWR(SYNOBIOS_IOC_MAGIC, 19, SYNO_AUTO_POWERON)
#define SYNOIO_GET_MEM_BYTE  _IOWR(SYNOBIOS_IOC_MAGIC, 20, MEMORY_BYTE)
#define SYNOIO_SET_MEM_BYTE  _IOWR(SYNOBIOS_IOC_MAGIC, 21, MEMORY_BYTE)
#define SYNOIO_SET_ALARM_LED  _IOWR(SYNOBIOS_IOC_MAGIC, 22, unsigned char)
#define SYNOIO_GET_HW_CAPABILITY	_IOWR(SYNOBIOS_IOC_MAGIC, 23, CAPABILITY)
#define SYNOIO_GET_FAN_NUM			_IOWR(SYNOBIOS_IOC_MAGIC, 24, int)
#define SYNOIO_GET_POWER_STATUS			_IOWR(SYNOBIOS_IOC_MAGIC, 25, POWER_INFO)
#define SYNOIO_SHUTDOWN_LOG			_IOWR(SYNOBIOS_IOC_MAGIC, 26, SYNO_SHUTDOWN_LOG)
#define SYNOIO_UNINITIALIZE			_IOWR(SYNOBIOS_IOC_MAGIC, 27, int)
#define SYNOIO_GET_SYS_STATUS		_IOWR(SYNOBIOS_IOC_MAGIC, 28, SYNO_SYS_STATUS)
#define SYNOIO_SET_SYS_STATUS		_IOWR(SYNOBIOS_IOC_MAGIC, 29, sys_comp_stat_t)
#define SYNOIO_GET_MODULE_TYPE		_IOWR(SYNOBIOS_IOC_MAGIC, 30, module_t)
#define SYNOIO_GET_BUZZER_CLEARED	_IOWR(SYNOBIOS_IOC_MAGIC, 31, unsigned char)
#define SYNOIO_GET_BACKPLANE_STATUS  _IOWR(SYNOBIOS_IOC_MAGIC, 32, BACKPLANE_STATUS)
#define SYNOIO_SET_UART2			_IOWR(SYNOBIOS_IOC_MAGIC, 33, unsigned char)
#define SYNOIO_GET_CPU_TEMPERATURE	_IOWR(SYNOBIOS_IOC_MAGIC, 34, SYNOCPUTEMP)
#define SYNOIO_SET_CPU_FAN_STATUS   _IOWR(SYNOBIOS_IOC_MAGIC, 35, FANSTATUS)
#define SYNOIO_SET_PHY_LED     _IOWR(SYNOBIOS_IOC_MAGIC, 36, SYNO_LED)
#define SYNOIO_SET_HDD_LED     _IOWR(SYNOBIOS_IOC_MAGIC, 37, SYNO_LED)
#define SYNOIO_SET_PWR_LED     _IOWR(SYNOBIOS_IOC_MAGIC, 38, SYNO_LED)
#define SYNOIO_PWM_CTL     _IOWR(SYNOBIOS_IOC_MAGIC, 39, SynoPWMCTL)
#define SYNOIO_CHECK_MICROP_ID     _IO(SYNOBIOS_IOC_MAGIC, 40)
//#define SYNOIO_GET_EUNIT_TYPE     _IOR(SYNOBIOS_IOC_MAGIC, 41, EUNIT_PWRON_TYPE) // Move into kernel
#define SYNOIO_SET_BUZZER_CLEAR	_IOWR(SYNOBIOS_IOC_MAGIC, 42, unsigned char)
#define SYNOIO_WRITE_MEM		_IOWR(SYNOBIOS_IOC_MAGIC, 43, SYNO_MEM_ACCESS)
#define SYNOIO_READ_MEM			_IOWR(SYNOBIOS_IOC_MAGIC, 44, SYNO_MEM_ACCESS)
#define SYNOIO_SET_AHA_LED     _IOWR(SYNOBIOS_IOC_MAGIC, 45, SYNO_AHA_LED)
#define SYNOIO_GET_COPY_BUTTON _IOWR(SYNOBIOS_IOC_MAGIC, 46, int) // for matching userspace usage, button pressed = 0, else = 1
#define SYNOIO_SET_AND_GET_UART2	_IOWR(SYNOBIOS_IOC_MAGIC, 47, unsigned char)
#define SYNOIO_GET_UART2			_IOWR(SYNOBIOS_IOC_MAGIC, 48, unsigned char)
#define SYNOIO_UART_DEBUG_CONTROL	_IOWR(SYNOBIOS_IOC_MAGIC, 49, unsigned char)
#define SYNOIO_GET_SYS_CURRENT _IOWR(SYNOBIOS_IOC_MAGIC, 50, unsigned long)
#define SYNOIO_SET_RP_FAN			_IOWR(SYNOBIOS_IOC_MAGIC, 51, unsigned char)

#define SYNOIO_POWER_OFF		_IOWR(SYNOBIOS_IOC_MAGIC, 65, int)

#define SYNOIO_MANUTIL_MODE       _IOWR(SYNOBIOS_IOC_MAGIC, 128, int)
#define SYNOIO_RECORD_EVENT       _IOWR(SYNOBIOS_IOC_MAGIC, 129, int)

#ifdef SYNO_HI_3535
#define SYNOIO_GPIO_PIN_READ      _IOWR(SYNOBIOS_IOC_MAGIC, 205, GPIO_PIN)
#define SYNOIO_GPIO_PIN_WRITE     _IOWR(SYNOBIOS_IOC_MAGIC, 206, GPIO_PIN)
#else
#define SYNOIO_GPIO_PIN_READ      _IOWR(SYNOBIOS_IOC_MAGIC, 205, int)
#define SYNOIO_GPIO_PIN_WRITE     _IOWR(SYNOBIOS_IOC_MAGIC, 206, int)
#endif

#define SYNOIO_RTC_READ           _IOWR(SYNOBIOS_IOC_MAGIC, 208, struct _SynoRtcPkt)
#define SYNOIO_RTC_READ_3         _IOWR(SYNOBIOS_IOC_MAGIC, 209, struct _SynoRtcPkt)
#define SYNOIO_SDA_SDL_READ       _IOWR(SYNOBIOS_IOC_MAGIC, 210, int)


#ifdef SYNO_BAD_SECTOR_DISK_DEBUG
#define DISK_BADSECTOR_ON      _IOWR(SYNOBIOS_IOC_MAGIC, 211, int)
#define DISK_BADSECTOR_OFF     _IOWR(SYNOBIOS_IOC_MAGIC, 212, int)
#define DISK_BADSECTOR_SET     _IOWR(SYNOBIOS_IOC_MAGIC, 213, struct disk_bs_map_ctlcmd)
#define DISK_BADSECTOR_GET     _IOWR(SYNOBIOS_IOC_MAGIC, 214, struct disk_bs_map_ctlcmd)
#define DISK_BADSECTOR_RESET   _IOWR(SYNOBIOS_IOC_MAGIC, 215, int)
#endif

#define HWMON_GET_SUPPORT              _IOWR(SYNOBIOS_IOC_MAGIC, 301, SYNO_HWMON_SUPPORT)
#define HWMON_GET_CPU_TEMPERATURE      _IOWR(SYNOBIOS_IOC_MAGIC, 302, SYNO_HWMON_SENSOR_TYPE)
#define HWMON_GET_FAN_SPEED_RPM        _IOWR(SYNOBIOS_IOC_MAGIC, 303, SYNO_HWMON_SENSOR_TYPE)
#define HWMON_GET_PSU_STATUS           _IOWR(SYNOBIOS_IOC_MAGIC, 304, SYNO_HWMON_SENSOR_TYPE)
#define HWMON_GET_SYS_VOLTAGE          _IOWR(SYNOBIOS_IOC_MAGIC, 305, SYNO_HWMON_SENSOR_TYPE)
#define HWMON_GET_SYS_THERMAL          _IOWR(SYNOBIOS_IOC_MAGIC, 306, SYNO_HWMON_SENSOR_TYPE)
#define HWMON_GET_HDD_BACKPLANE        _IOWR(SYNOBIOS_IOC_MAGIC, 307, SYNO_HWMON_SENSOR_TYPE)
#define HWMON_GET_SYS_CURRENT          _IOWR(SYNOBIOS_IOC_MAGIC, 308, SYNO_HWMON_SENSOR_TYPE)

/* Move into kernel
#define SYNOIO_SUPERIO_READ		_IOWR(SYNOBIOS_IOC_MAGIC, 216, SYNO_SUPERIO_PACKAGE)
#define SYNOIO_SUPERIO_WRITE		_IOWR(SYNOBIOS_IOC_MAGIC, 217, SYNO_SUPERIO_PACKAGE)
#define SYNOIO_IS_FULLY_SUPPORT_EUP	_IOWR(SYNOBIOS_IOC_MAGIC, 218, SYNO_EUP_SUPPORT)
*/

#define SYNOIO_SET_OK_TO_REMOVE_LED  _IOWR(SYNOBIOS_IOC_MAGIC, 219, unsigned char)
#define SYNOIO_CHECK_DISK_INTF       _IOWR(SYNOBIOS_IOC_MAGIC, 220, SYNO_DISK_INTF_INFO)

/*
#define SYNOIO_HWSTATUS		_IOR('A', 102, struct _Synohw)
#define SYNOIO_TESTING		_IOW('P', 104, int)
#define SYNOIO_SERIAL		_IOR('A', 105, int)
#define SYNOIO_HDBACK		_IOW('P', 106, int)
#define SYNOIO_SYNOVER		_IOR('A', 107, unsigned long)
#define SYNOIO_GETSERIALNUM	_IOR('A', 108, off_t)
#define SYNOIO_SETHWSTATUS	_IOW('P', 109, HWBLOCK)
#define SYNOIO_HWHDSUPPORT	_IOR('A', 110, int)
#define SYNOIO_HWTHERMALSUPPORT	_IOR('A', 111, int)
#define SYNOIO_POWEROFF		_IO('A', 114)
#define SYNOIO_SETHWSTATUS_INIT _IOW('P', 111, HWBLOCK)
#define SYNOIO_HWSTATUS_CUSTOM  _IOR('A', 112, struct _Synohw_custom)
#define SYNOIO_SET_EVENT        _IOW('P', 113, u_int)
#define SYNOIO_BIOSVER		_IOWR('P', 115, BVS)
*/
#define SCEM_IOC_TESTING_DISABLE_TEST		0x00
#define SCEM_IOC_TESTING_ENABLE_TEST		0x01

#define SCEM_IOC_TESTING_ACPOWER_ON		0x10
#define SCEM_IOC_TESTING_ACPOWER_OFF		0x11

#define SCEM_IOC_TESTING_SET_OVERHEATING_0	0x20
#define SCEM_IOC_TESTING_SET_OVERHEATING_1	0x21
#define SCEM_IOC_TESTING_SET_OVERHEATING_2	0x22
#define SCEM_IOC_TESTING_SET_OVERHEATING_3	0x23

#define SCEM_IOC_TESTING_SET_MEMORY_ECC_0	0x30
#define SCEM_IOC_TESTING_SET_MEMORY_ECC_1	0x31
#define SCEM_IOC_TESTING_SET_MEMORY_ECC_2	0x32

#define SCEM_IOC_TESTING_SET_BUTTON_0		0x40
#define SCEM_IOC_TESTING_SET_BUTTON_1		0x41
#define SCEM_IOC_TESTING_SET_BUTTON_2		0x42
#define SCEM_IOC_TESTING_SET_BUTTON_3		0x43
#define SCEM_IOC_TESTING_SET_BUTTON_4		0x44
#define SCEM_IOC_TESTING_SET_BUTTON_5		0x45
#define SCEM_IOC_TESTING_SET_BUTTON_6		0x46
#define SCEM_IOC_TESTING_SET_BUTTON_7		0x47
#define SCEM_IOC_TESTING_SET_BUTTON_8		0x48

#define SCEM_IOC_TESTING_RM_HDA			0x51
#define SCEM_IOC_TESTING_RM_HDB			0x52
#define SCEM_IOC_TESTING_RM_HDC			0x53
#define SCEM_IOC_TESTING_RM_HDD			0x54
#define SCEM_IOC_TESTING_RM_HDE			0x55
#define SCEM_IOC_TESTING_RM_HDF			0x56
#define SCEM_IOC_TESTING_RM_HDG			0x57
#define SCEM_IOC_TESTING_RM_HDH			0x58

#define SCEM_IOC_TESTING_PLUG_HDA		0x61
#define SCEM_IOC_TESTING_PLUG_HDB		0x62
#define SCEM_IOC_TESTING_PLUG_HDC		0x63
#define SCEM_IOC_TESTING_PLUG_HDD		0x64
#define SCEM_IOC_TESTING_PLUG_HDE		0x65
#define SCEM_IOC_TESTING_PLUG_HDF		0x66
#define SCEM_IOC_TESTING_PLUG_HDG		0x67
#define SCEM_IOC_TESTING_PLUG_HDH		0x68

#define ACPOWER_FAILURE		1

#define THERMAL_NORMAL		0
#define THERMAL_WARM		1
#define THERMAL_HEAT		2
#define THERMAL_CRITICAL	3

#define NO_MEM_ECC		0
#define MEM_ECC_ONCE		1
#define MEM_ECC_TWICE_OR_MORE	2

#define THERMAL_STAGE(a)				\
        ((a >= iThermal3) ? THERMAL_CRITICAL :		\
        ((a >= iThermal2) ? THERMAL_HEAT :		\
        ((a >= iThermal1) ? THERMAL_WARM : THERMAL_NORMAL)))

#define F_GET_ACPOWER(a)	(a->AcPowerStatus)
#define F_GET_THERMAL(a)	(a->ThermalStage)
#define F_GET_BUTTON(a)		(a->ButtonStatus)
#define F_GET_MEMORY(a)		(a->MemEccCount)

#define F_GET_1_BUTTON(a)	((a->ButtonStatus)&0x0001)
#define F_GET_IPBUTTON(a)	((a->ButtonStatus)&0x0001)
#define F_GET_2_BUTTON(a)	((a->ButtonStatus)&0x0002)
#define F_GET_BACKUP_BUTTON(a)	((a->ButtonStatus)&0x0002)
#define F_GET_3_BUTTON(a)	((a->ButtonStatus)&0x0004)
#define F_GET_4_BUTTON(a)	((a->ButtonStatus)&0x0008)
#define F_GET_5_BUTTON(a)	((a->ButtonStatus)&0x0010)
#define F_GET_6_BUTTON(a)	((a->ButtonStatus)&0x0020)
#define F_GET_7_BUTTON(a)	((a->ButtonStatus)&0x0040)
#define F_GET_8_BUTTON(a)	((a->ButtonStatus)&0x0080)

#define EQ_ACPOWER_STATUS(a,b)	((a->AcPowerStatus)==(b->AcPowerStatus))
#define EQ_THERMAL_STATUS(a,b)	(THERMAL_STAGE(a->ThermalStage)==THERMAL_STAGE(b->ThermalStage))
#define EQ_MEMORY_STATUS(a,b)	((a->MemEccCount)==(b->MemEccCount))
#define EQ_BUTTON_STATUS(a,b)	((a->ButtonStatus)==(b->ButtonStatus))
#define EQ_HD_STATUS(a, b)	((a->HardDiskStatus) == (b->HardDiskStatus))
#define EQ_FAN_STATUS(a, b)	((a->FanFaultStatus) == (b->FanFaultStatus))

#define EQ_BUTTON_1_4(a, b)	((a->ButtonStatus&0x000f)==(b->ButtonStatus&0x000f))
#define EQ_BUTTON_1_2(a, b)	((a->ButtonStatus&0x0003)==(b->ButtonStatus&0x0003))
#define EQ_BUTTON_3_4(a, b)	((a->ButtonStatus&0x000c)==(b->ButtonStatus&0x000c))
#define EQ_BUTTON_5_8(a, b)	((a->ButtonStatus&0x00f0)==(b->ButtonStatus&0x00f0))
#define EQ_BUTTON_5_6(a, b)	((a->ButtonStatus&0x0030)==(b->ButtonStatus&0x0030))
#define EQ_BUTTON_7_8(a, b)	((a->ButtonStatus&0x00c0)==(b->ButtonStatus&0x00c0))
#define EQ_BUTTON_1(a, b)	((a->ButtonStatus&0x0001)==(b->ButtonStatus&0x0001))
#define EQ_BUTTON_2(a, b)	((a->ButtonStatus&0x0002)==(b->ButtonStatus&0x0002))
#define EQ_BUTTON_3(a, b)	((a->ButtonStatus&0x0004)==(b->ButtonStatus&0x0004))
#define EQ_BUTTON_4(a, b)	((a->ButtonStatus&0x0008)==(b->ButtonStatus&0x0008))
#define EQ_BUTTON_5(a, b)	((a->ButtonStatus&0x0010)==(b->ButtonStatus&0x0010))
#define EQ_BUTTON_6(a, b)	((a->ButtonStatus&0x0020)==(b->ButtonStatus&0x0020))
#define EQ_BUTTON_7(a, b)	((a->ButtonStatus&0x0040)==(b->ButtonStatus&0x0040))
#define EQ_BUTTON_8(a, b)	((a->ButtonStatus&0x0080)==(b->ButtonStatus&0x0080))

#define F_GET_HD_STATUS_A(a)	((a->HardDiskStatus&0x0001))
#define F_GET_HD_STATUS_B(a)	((a->HardDiskStatus&0x0002))
#define F_GET_HD_STATUS_C(a)	((a->HardDiskStatus&0x0004))
#define F_GET_HD_STATUS_D(a)	((a->HardDiskStatus&0x0008))
#define F_GET_HD_STATUS_E(a)	((a->HardDiskStatus&0x0010))
#define F_GET_HD_STATUS_F(a)	((a->HardDiskStatus&0x0020))
#define F_GET_HD_STATUS_G(a)	((a->HardDiskStatus&0x0040))
#define F_GET_HD_STATUS_H(a)	((a->HardDiskStatus&0x0080))

#define EQ_HD_STATUS_A_D(a,b)   ((a->HardDiskStatus&0x000f)==(b->HardDiskStatus&0x000f))
#define EQ_HD_STATUS_E_H(a,b)   ((a->HardDiskStatus&0x00f0)==(b->HardDiskStatus&0x00f0))
#define EQ_HD_STATUS_A_B(a,b)   ((a->HardDiskStatus&0x0003)==(b->HardDiskStatus&0x0003))
#define EQ_HD_STATUS_C_D(a,b)   ((a->HardDiskStatus&0x000c)==(b->HardDiskStatus&0x000c))
#define EQ_HD_STATUS_E_F(a,b)   ((a->HardDiskStatus&0x0030)==(b->HardDiskStatus&0x0030))
#define EQ_HD_STATUS_G_H(a,b)   ((a->HardDiskStatus&0x00c0)==(b->HardDiskStatus&0x00c0))
#define EQ_HD_STATUS_A(a,b)	((a->HardDiskStatus&0x0001)==(b->HardDiskStatus&0x0001))
#define EQ_HD_STATUS_B(a,b)	((a->HardDiskStatus&0x0002)==(b->HardDiskStatus&0x0002))
#define EQ_HD_STATUS_C(a,b)	((a->HardDiskStatus&0x0004)==(b->HardDiskStatus&0x0004))
#define EQ_HD_STATUS_D(a,b)	((a->HardDiskStatus&0x0008)==(b->HardDiskStatus&0x0008))
#define EQ_HD_STATUS_E(a,b)	((a->HardDiskStatus&0x0010)==(b->HardDiskStatus&0x0010))
#define EQ_HD_STATUS_F(a,b)	((a->HardDiskStatus&0x0020)==(b->HardDiskStatus&0x0020))
#define EQ_HD_STATUS_G(a,b)	((a->HardDiskStatus&0x0040)==(b->HardDiskStatus&0x0040))
#define EQ_HD_STATUS_H(a,b)	((a->HardDiskStatus&0x0080)==(b->HardDiskStatus&0x0080))

#define PMEV_SYNO_EVENT0	0x0200
#define PMEV_SYNO_EVENT1	0x0201
#define PMEV_SYNO_EVENT2	0x0202
#define PMEV_SYNO_EVENT3	0x0203
#define PMEV_SYNO_EVENT4	0x0204
#define PMEV_SYNO_EVENT5	0x0205
#define PMEV_SYNO_EVENT6	0x0206
#define PMEV_SYNO_EVENT7	0x0207
#define PMEV_SYNO_EVENT8	0x0208
#define PMEV_SYNO_EVENT9	0x0209

#define VMAX              32
#define MAX_HD_NUM        16
#define HD_NUM_MAX        16
#define FAN_NUM_MAX       16
#define POWER_NUM_MAX     16
#define BATTERY_NUM_MAX   16
#define LP3943_NUM_MAX     8


/*************************************************************************
 * SYNOHWWxternalControl message Id
 ************************************************************************/

#define SYNO_PURE_MESSAGE               0x0000
#define SYNO_DISABLE_CRITICAL           0x0100
#define SYNO_ENABLE_CRITICAL            0x0101
#define SYNO_E_MAIL_SUCCESS             0x0102
#define SYNO_E_MAIL_FAILED              0x0103
#define SYNO_DISABLE_IDENTIFY           0x0200
#define SYNO_ENABLE_IDENTIFY            0x0300
#define SYNO_BEEPER_MUTE                0x0400
#define SYNO_BEEPER_BEEPBEEP            0x0500
#define SYNO_CANCEL_DISK_FULL           0x0800
#define SYNO_DISK_FULL                  0x0900
#define SYNO_CANCEL_VOLUME_FULL         0x8000
#define SYNO_VOLUME_FULL                0x9000
#define SYNO_DISK_NORMAL                0x1000
#define SYNO_DISK_FAILURE               0x1100
#define SYNO_DISK_REBUILD               0x1200
#define SYNO_DISK_ABSENT                0x1300
#define SYNO_DISK_INITIAL               0x1400
#define SYNO_DISK_EMPTY                 0x1500
#define SYNO_DISK_BAD_SECTOR            0x1600
#define SYNO_DISK_BAD_SYS               0x1700
#define SYNO_DISK_BAD_SYSNSEC           0x1800
#define SYNO_CPU_OVERHEAT               0x2101
#define SYNO_CPU_NORMAL                 0x2001
#define SYNO_MEM_ECC                    0x2104
#define SYNO_FAN_FAULT                  0x3000
#define SYNO_POWER_FAULT                0x3100
#define SYNO_BATTERY_FAILURE            0x3200
#define SYNO_BATTERY_REMOVED            0x3300
#define SYNO_FAN_RECOVERY               0x3400
#define SYNO_POWER_RECOVERY             0x3500
#define SYNO_NOP_SUCCESS                0x3600
#define SYNO_NOP_FAILURE                0x3700
#define SYNO_VOL_NORMAL                 0x3800
#define SYNO_VOL_BUILT                  0x3900
#define SYNO_VOL_FAILED                 0x3a00
#define SYNO_VOL_CREATE                 0x3b00
#define SYNO_VOL_REMOVE                 0x3c00
#define SYNO_SYS_MSG                    0x4000
#define SYNO_SYS_BOOT                   0x4001
#define SYNO_SYS_RUN                    0x4002
#define SYNO_SYS_SHUTDOWN               0x4003
#define SYNO_SYS_NO_SYSTEM              0x4004
#define SYNO_SYS_NODISK                 SYNO_SYS_NO_SYSTEM  // NODISK is alias of NO_SYSTEM
#define SYNO_SYS_WAIT_RESET             0x4005
#define SYNO_SYS_FACTORY_DEFAULT        0x4006
#define SYNO_LED_USB_COPY_NONE          0x5100
#define SYNO_LED_USB_COPY_STEADY        0x5101
#define SYNO_LED_USB_COPY_BLINK         0x5102
#define SYNO_LED_USB_EJECT_BLINK        0x5103
#define SYNO_LED_HDD_GS                 0x5200
#define SYNO_LED_HDD_AS                 0x5201
#define SYNO_LED_HDD_AB                 0x5202
#define SYNO_LED_HDD_OFF                0x5203
#define SYNO_LED_HDD_GB                 0x5204
#define SYNO_LED_ALARM_ON				0x5205
#define SYNO_LED_ALARM_BLINKING			0x5206
#define SYNO_LED_ALARM_OFF				0x5207
#define SYNO_BEEP_OFF                   0x5300
#define SYNO_BEEP_ON                    0x5301
#define SYNO_BEEP_200MS                 0x5302
#define SYNO_POWER_OFF                  0x5400
#define SYNO_RCPOWER_ON                 0x5500
#define SYNO_RCPOWER_OFF                0x5600
#define SYNO_AUTO_POWERON_ON            0x5700
#define SYNO_AUTO_POWERON_OFF           0x5701
#define SYNO_UART2_FANCHEK_ON           0x5800
#define SYNO_UART2_FANCHEK_OFF          0x5801
#define SYNO_LED_USBSTATION_DISK_GREEN  0x5900
#define SYNO_LED_USBSTATION_DISK_ORANGE 0x5901
#define SYNO_LED_USBSTATION_POWER       0x5902
#define SYNO_LED_USBSTATION_MEMTEST_LED		0x5903

/*add by chchou : moved from synobios.c*/
/*int SYNOBiosSetEvent(u_int event_type);
int	ErrSYNOHardwareConfigure __P((int , int *));
*/
/**/
/* Eval pin value considering polarity, output HIGH or LOW
	P V (!P == !V)
	0 0  1
	1 0  0
	0 1  0
	1 1  1 */
#define EVAL_PIN_VAL(polarity, value) (!(polarity) == !(value))

struct synobios_ops {
	struct module	*owner;
	int		(*get_brand)(void);
	int		(*get_model)(void);
	int		(*get_cpld_version)(void);
	int		(*get_rtc_time)(struct _SynoRtcTimePkt *);
	int		(*set_rtc_time)(struct _SynoRtcTimePkt *);
	int		(*get_fan_status)(int, FAN_STATUS *);
	int		(*set_fan_status)(FAN_STATUS, FAN_SPEED);
	int		(*get_sys_temperature)(struct _SynoThermalTemp *);
	int		(*get_cpu_temperature)(struct _SynoCpuTemp *);
	#if defined(CONFIG_SYNO_PORT_MAPPING_V2)
	int		(*set_disk_led)(DISKLEDSTATUS*);
	#else /* CONFIG_SYNO_PORT_MAPPING_V2 */
	int		(*set_disk_led)(int, SYNO_DISK_LED);
	#endif /* CONFIG_SYNO_PORT_MAPPING_V2 */
	int		(*set_power_led)(SYNO_LED);
	int		(*get_cpld_reg)(CPLDREG *);
	int		(*set_mem_byte)(MEMORY_BYTE *);
	int		(*get_mem_byte)(MEMORY_BYTE *);
	int		(*set_gpio_pin)(GPIO_PIN *);
	int		(*get_gpio_pin)(GPIO_PIN *);
	int		(*set_gpio_blink)(GPIO_PIN *);
	int		(*set_auto_poweron)(SYNO_AUTO_POWERON *);
	int		(*get_auto_poweron)(SYNO_AUTO_POWERON *);
	int		(*init_auto_poweron)(void);
	int		(*uninit_auto_poweron)(void);
	int		(*set_alarm_led)(unsigned char);
	int		(*get_buzzer_cleared)(unsigned char *buzzer_cleared);
	int		(*set_buzzer_clear)(unsigned char buzzer_clear);
	int		(*get_power_status)(POWER_INFO *);
	int		(*get_backplane_status)(BACKPLANE_STATUS *);
	int		(*module_type_init)(struct synobios_ops *);
	int		(*uninitialize)(void);
	int		(*set_cpu_fan_status)(FAN_STATUS, FAN_SPEED);
	int   (*set_phy_led)(SYNO_LED);
	int   (*set_hdd_led)(SYNO_LED);
	int		(*pwm_ctl)(SynoPWMCTL *);
	int		(*check_microp_id)(const struct synobios_ops *);
	int		(*set_microp_id)(void);
	int		(*get_superio)(SYNO_SUPERIO_PACKAGE *);
	int		(*set_superio)(SYNO_SUPERIO_PACKAGE *);
	int		(*exdisplay_handler)(struct _SynoMsgPkt *);
	int		(*read_memory)(SYNO_MEM_ACCESS*);
	int		(*write_memory)(SYNO_MEM_ACCESS*);
	void	        (*get_cpu_info)(SYNO_CPU_INFO*, const unsigned int);
	int             (*set_aha_led)(struct synobios_ops *, SYNO_AHA_LED);
	int             (*get_copy_button_status)(void); // for matching userspace usage, button pressed = 0, else = 1
	int             (*hwmon_get_fan_speed_rpm)(SYNO_HWMON_SENSOR_TYPE *);
	int             (*hwmon_get_psu_status)(SYNO_HWMON_SENSOR_TYPE *, int);
	int             (*hwmon_get_sys_voltage)(SYNO_HWMON_SENSOR_TYPE *);
	int             (*hwmon_get_backplane_status)(SYNO_HWMON_SENSOR_TYPE *);
	int             (*hwmon_get_sys_thermal)(SYNO_HWMON_SENSOR_TYPE *);
	int             (*hwmon_get_sys_current)(SYNO_HWMON_SENSOR_TYPE *);
	int 		(*set_ok_to_remove_led)(unsigned char ledON);
	int		(*get_sys_current)(unsigned long*);
	int             (*get_disk_intf)(SYNO_DISK_INTF_INFO *);
	int		(*set_rp_fan)(unsigned char);
};

/**************************/
#define IXP425

PRODUCT_MODEL synobios_getmodel(void);

#define PCI_VENDOR_SYNOLOGY 0x7053

#define PCI_DEVICE_E10G18T1 0x1001
#define PCI_DEVICE_E10G18T2 0x1002
#define PCI_DEVICE_E10G21F2 0x1003
#define PCI_DEVICE_E25G21F2 0x1004
#define PCI_DEVICE_E10G30T1 0x1005
#define PCI_DEVICE_E10G22T1_MINI 0x1006
#define PCI_DEVICE_E10G30T2_BROADCOM 0x1008
#define PCI_DEVICE_E10G22T1_MINI_AQC107 0x1009
#define PCI_DEVICE_E10G30T2_QLOGIC 0x100a
#define PCI_DEVICE_E10G30F2 0x100b
#define PCI_DEVICE_E25G30F2 0x100c
#define PCI_DEVICE_E10G30T1_AQC113 0x100d
#define PCI_DEVICE_E10G22T1_MINI_AQC113 0x100e
#define PCI_DEVICE_E10M20T1 0x2002

#define SZ_SYNOLOGY_M2D17 "M2D17"
#define SZ_SYNOLOGY_M2D18 "M2D18"
#define SZ_SYNOLOGY_M2D20 "M2D20"
#define SZ_SYNOLOGY_E10G18T1 "E10G18-T1"
#define SZ_SYNOLOGY_E10G18T2 "E10G18-T2"
#define SZ_SYNOLOGY_E10G21F2 "E10G21-F2"
#define SZ_SYNOLOGY_E25G21F2 "E25G21-F2"
#define SZ_SYNOLOGY_E10G30T1 "E10G30-T1"
#define SZ_SYNOLOGY_E10G30T2 "E10G30-T2"
#define SZ_SYNOLOGY_E10G30F2 "E10G30-F2"
#define SZ_SYNOLOGY_E25G30F2 "E25G30-F2"
#define SZ_SYNOLOGY_E10G22T1_MINI "E10G22-T1-Mini"
#define SZ_SYNOLOGY_E10M20T1 "E10M20-T1"
#define SZ_SYNOLOGY_FX2422N "FX2422N"

#define __SYNOEVENTCALL_PARM1(parms) parms[0]
#define __SYNOEVENTCALL_PARM2(parms) \
	__SYNOEVENTCALL_PARM1(parms), parms[1]
#define __SYNOEVENTCALL_PARM3(parms) \
	__SYNOEVENTCALL_PARM2(parms), parms[2]
#define __SYNOEVENTCALL_PARM4(parms) \
	__SYNOEVENTCALL_PARM3(parms), parms[3]
#define __SYNOEVENTCALL_PARM5(parms) \
	__SYNOEVENTCALL_PARM4(parms), parms[4]

#endif /* __SYNOBIOS_OEM_H_ */
