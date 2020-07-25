/**
 ******************************************************************************
 * @addtogroup UAVObjects OpenPilot UAVObjects
 * @{
 * @addtogroup SystemIdentSettings SystemIdentSettings
 * @brief The input to and results of the PID tuning.
 *
 * Autogenerated files and functions for SystemIdentSettings Object
 *
 * @{
 *
 * @file       systemidentsettings.h
 * @author     The OpenPilot Team, http://www.openpilot.org Copyright (C) 2010-2013.
 * @brief      Implementation of the SystemIdentSettings object. This file has been
 *             automatically generated by the UAVObjectGenerator.
 *
 * @note       Object definition file: systemidentsettings.xml.
 *             This is an automatically generated file.
 *             DO NOT modify manually.
 *
 * @see        The GNU Public License (GPL) Version 3
 *
 *****************************************************************************/
/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 * or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License
 * for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 */

#ifndef SYSTEMIDENTSETTINGS_H
#define SYSTEMIDENTSETTINGS_H
#include <stdbool.h>

/* Object constants */
#define SYSTEMIDENTSETTINGS_OBJID 0xE19AC6C2
#define SYSTEMIDENTSETTINGS_ISSINGLEINST 1
#define SYSTEMIDENTSETTINGS_ISSETTINGS 1
#define SYSTEMIDENTSETTINGS_ISPRIORITY 0
#define SYSTEMIDENTSETTINGS_NUMBYTES sizeof(SystemIdentSettingsData)

/* Generic interface functions */
int32_t SystemIdentSettingsInitialize();
UAVObjHandle SystemIdentSettingsHandle();
void SystemIdentSettingsSetDefaults(UAVObjHandle obj, uint16_t instId);

/* Field Tau information */

/* Field Beta information */

// Array element names for field Beta
typedef enum {
    SYSTEMIDENTSETTINGS_BETA_ROLL=0,
    SYSTEMIDENTSETTINGS_BETA_PITCH=1,
    SYSTEMIDENTSETTINGS_BETA_YAW=2
} SystemIdentSettingsBetaElem;

// Number of elements for field Beta
#define SYSTEMIDENTSETTINGS_BETA_NUMELEM 3

/* Field YawToRollPitchPIDRatioMin information */

/* Field YawToRollPitchPIDRatioMax information */

/* Field DerivativeFactor information */

/* Field OuterLoopKpSoftClamp information */

/* Field SmoothQuickValue information */

/* Field GyroReadTimeAverage information */

/* Field DampMin information */

/* Field DampRate information */

/* Field DampMax information */

/* Field NoiseMin information */

/* Field NoiseRate information */

/* Field NoiseMax information */

/* Field CalculateYaw information */

// Enumeration options for field CalculateYaw
typedef enum __attribute__ ((__packed__)) {
    SYSTEMIDENTSETTINGS_CALCULATEYAW_FALSE=0,
    SYSTEMIDENTSETTINGS_CALCULATEYAW_TRUELIMITTORATIO=1,
    SYSTEMIDENTSETTINGS_CALCULATEYAW_TRUEIGNORELIMIT=2
} SystemIdentSettingsCalculateYawOptions;

/* Field DestinationPidBank information */

// Enumeration options for field DestinationPidBank
typedef enum __attribute__ ((__packed__)) {
    SYSTEMIDENTSETTINGS_DESTINATIONPIDBANK_BANK1=0,
    SYSTEMIDENTSETTINGS_DESTINATIONPIDBANK_BANK2=1,
    SYSTEMIDENTSETTINGS_DESTINATIONPIDBANK_BANK3=2
} SystemIdentSettingsDestinationPidBankOptions;

/* Field TuningDuration information */

/* Field ThrustControl information */

// Enumeration options for field ThrustControl
typedef enum __attribute__ ((__packed__)) {
    SYSTEMIDENTSETTINGS_THRUSTCONTROL_MANUAL=0,
    SYSTEMIDENTSETTINGS_THRUSTCONTROL_ALTITUDEVARIO=1
} SystemIdentSettingsThrustControlOptions;

/* Field SmoothQuickSource information */

// Enumeration options for field SmoothQuickSource
typedef enum __attribute__ ((__packed__)) {
    SYSTEMIDENTSETTINGS_SMOOTHQUICKSOURCE_DISABLED=0,
    SYSTEMIDENTSETTINGS_SMOOTHQUICKSOURCE_ACCESSORY0=1,
    SYSTEMIDENTSETTINGS_SMOOTHQUICKSOURCE_ACCESSORY1=2,
    SYSTEMIDENTSETTINGS_SMOOTHQUICKSOURCE_ACCESSORY2=3,
    SYSTEMIDENTSETTINGS_SMOOTHQUICKSOURCE_ACCESSORY3=4,
    SYSTEMIDENTSETTINGS_SMOOTHQUICKSOURCE_FMSTOGGLE3POS=5,
    SYSTEMIDENTSETTINGS_SMOOTHQUICKSOURCE_FMSTOGGLE5POS=6,
    SYSTEMIDENTSETTINGS_SMOOTHQUICKSOURCE_FMSTOGGLE7POS=7
} SystemIdentSettingsSmoothQuickSourceOptions;

/* Field DisableSanityChecks information */

// Enumeration options for field DisableSanityChecks
typedef enum __attribute__ ((__packed__)) {
    SYSTEMIDENTSETTINGS_DISABLESANITYCHECKS_FALSE=0,
    SYSTEMIDENTSETTINGS_DISABLESANITYCHECKS_TRUE=1
} SystemIdentSettingsDisableSanityChecksOptions;

/* Field Complete information */

// Enumeration options for field Complete
typedef enum __attribute__ ((__packed__)) {
    SYSTEMIDENTSETTINGS_COMPLETE_FALSE=0,
    SYSTEMIDENTSETTINGS_COMPLETE_TRUE=1
} SystemIdentSettingsCompleteOptions;



typedef struct __attribute__ ((__packed__)) {
    float Roll;
    float Pitch;
    float Yaw;
}  SystemIdentSettingsBetaData ;
typedef struct __attribute__ ((__packed__)) {
    float array[3];
}  SystemIdentSettingsBetaDataArray ;
#define SystemIdentSettingsBetaToArray( var ) UAVObjectFieldToArray( SystemIdentSettingsBetaData, var )


/*
 * Packed Object data (unaligned).
 * Should only be used where 4 byte alignment can be guaranteed
 * (eg a single instance on the heap)
 */
typedef struct {
        float Tau;
    SystemIdentSettingsBetaData Beta;
    float YawToRollPitchPIDRatioMin;
    float YawToRollPitchPIDRatioMax;
    float DerivativeFactor;
    float OuterLoopKpSoftClamp;
    float SmoothQuickValue;
    float GyroReadTimeAverage;
    uint8_t DampMin;
    uint8_t DampRate;
    uint8_t DampMax;
    uint8_t NoiseMin;
    uint8_t NoiseRate;
    uint8_t NoiseMax;
    SystemIdentSettingsCalculateYawOptions CalculateYaw;
    SystemIdentSettingsDestinationPidBankOptions DestinationPidBank;
    uint8_t TuningDuration;
    SystemIdentSettingsThrustControlOptions ThrustControl;
    SystemIdentSettingsSmoothQuickSourceOptions SmoothQuickSource;
    SystemIdentSettingsDisableSanityChecksOptions DisableSanityChecks;
    SystemIdentSettingsCompleteOptions Complete;

} __attribute__((packed)) SystemIdentSettingsDataPacked;

/*
 * Packed Object data.
 * Alignment is forced to 4 bytes so as to avoid the potential for CPU usage faults
 * on Cortex M4F during load/store of float UAVO fields
 */
typedef SystemIdentSettingsDataPacked __attribute__((aligned(4))) SystemIdentSettingsData;

void SystemIdentSettingsDataOverrideDefaults(SystemIdentSettingsData * data);

/* Typesafe Object access functions */
static inline int32_t SystemIdentSettingsGet(SystemIdentSettingsData * dataOut) {
    return UAVObjGetData(SystemIdentSettingsHandle(), dataOut);
}
static inline int32_t SystemIdentSettingsSet(const SystemIdentSettingsData * dataIn) {
    return UAVObjSetData(SystemIdentSettingsHandle(), dataIn);
}
static inline int32_t SystemIdentSettingsInstGet(uint16_t instId, SystemIdentSettingsData * dataOut) {
    return UAVObjGetInstanceData(SystemIdentSettingsHandle(), instId, dataOut);
}
static inline int32_t SystemIdentSettingsInstSet(uint16_t instId, const SystemIdentSettingsData * dataIn) {
    return UAVObjSetInstanceData(SystemIdentSettingsHandle(), instId, dataIn);
}
static inline int32_t SystemIdentSettingsConnectQueue(xQueueHandle queue) {
    return UAVObjConnectQueue(SystemIdentSettingsHandle(), queue, EV_MASK_ALL_UPDATES);
}
static inline int32_t SystemIdentSettingsConnectCallback(UAVObjEventCallback cb) {
    return UAVObjConnectCallback(SystemIdentSettingsHandle(), cb, EV_MASK_ALL_UPDATES, false);
}
static inline int32_t SystemIdentSettingsConnectFastCallback(UAVObjEventCallback cb) {
    return UAVObjConnectCallback(SystemIdentSettingsHandle(), cb, EV_MASK_ALL_UPDATES, true);
}
static inline uint16_t SystemIdentSettingsCreateInstance() {
    return UAVObjCreateInstance(SystemIdentSettingsHandle());
}
static inline void SystemIdentSettingsRequestUpdate() {
    UAVObjRequestUpdate(SystemIdentSettingsHandle());
}
static inline void SystemIdentSettingsRequestInstUpdate(uint16_t instId) {
    UAVObjRequestInstanceUpdate(SystemIdentSettingsHandle(), instId);
}
static inline void SystemIdentSettingsUpdated() {
    UAVObjUpdated(SystemIdentSettingsHandle());
}
static inline void SystemIdentSettingsInstUpdated(uint16_t instId) {
    UAVObjInstanceUpdated(SystemIdentSettingsHandle(), instId);
}
static inline void SystemIdentSettingsLogging() {
    UAVObjLogging(SystemIdentSettingsHandle());
}
static inline void SystemIdentSettingsInstLogging(uint16_t instId) {
    UAVObjInstanceLogging(SystemIdentSettingsHandle(), instId);
}
static inline int32_t SystemIdentSettingsGetMetadata(UAVObjMetadata * dataOut) {
    return UAVObjGetMetadata(SystemIdentSettingsHandle(), dataOut);
}
static inline int32_t SystemIdentSettingsSetMetadata(const UAVObjMetadata * dataIn) {
    return UAVObjSetMetadata(SystemIdentSettingsHandle(), dataIn);
}
static inline int8_t SystemIdentSettingsReadOnly() {
    return UAVObjReadOnly(SystemIdentSettingsHandle());
}

/* Set/Get functions */
extern void SystemIdentSettingsTauSet(float *NewTau);
extern void SystemIdentSettingsTauGet(float *NewTau);
extern void SystemIdentSettingsBetaSet(SystemIdentSettingsBetaData *NewBeta);
extern void SystemIdentSettingsBetaGet(SystemIdentSettingsBetaData *NewBeta);
extern void SystemIdentSettingsBetaArraySet(float *NewBeta);
extern void SystemIdentSettingsBetaArrayGet(float *NewBeta);
extern void SystemIdentSettingsYawToRollPitchPIDRatioMinSet(float *NewYawToRollPitchPIDRatioMin);
extern void SystemIdentSettingsYawToRollPitchPIDRatioMinGet(float *NewYawToRollPitchPIDRatioMin);
extern void SystemIdentSettingsYawToRollPitchPIDRatioMaxSet(float *NewYawToRollPitchPIDRatioMax);
extern void SystemIdentSettingsYawToRollPitchPIDRatioMaxGet(float *NewYawToRollPitchPIDRatioMax);
extern void SystemIdentSettingsDerivativeFactorSet(float *NewDerivativeFactor);
extern void SystemIdentSettingsDerivativeFactorGet(float *NewDerivativeFactor);
extern void SystemIdentSettingsOuterLoopKpSoftClampSet(float *NewOuterLoopKpSoftClamp);
extern void SystemIdentSettingsOuterLoopKpSoftClampGet(float *NewOuterLoopKpSoftClamp);
extern void SystemIdentSettingsSmoothQuickValueSet(float *NewSmoothQuickValue);
extern void SystemIdentSettingsSmoothQuickValueGet(float *NewSmoothQuickValue);
extern void SystemIdentSettingsGyroReadTimeAverageSet(float *NewGyroReadTimeAverage);
extern void SystemIdentSettingsGyroReadTimeAverageGet(float *NewGyroReadTimeAverage);
extern void SystemIdentSettingsDampMinSet(uint8_t *NewDampMin);
extern void SystemIdentSettingsDampMinGet(uint8_t *NewDampMin);
extern void SystemIdentSettingsDampRateSet(uint8_t *NewDampRate);
extern void SystemIdentSettingsDampRateGet(uint8_t *NewDampRate);
extern void SystemIdentSettingsDampMaxSet(uint8_t *NewDampMax);
extern void SystemIdentSettingsDampMaxGet(uint8_t *NewDampMax);
extern void SystemIdentSettingsNoiseMinSet(uint8_t *NewNoiseMin);
extern void SystemIdentSettingsNoiseMinGet(uint8_t *NewNoiseMin);
extern void SystemIdentSettingsNoiseRateSet(uint8_t *NewNoiseRate);
extern void SystemIdentSettingsNoiseRateGet(uint8_t *NewNoiseRate);
extern void SystemIdentSettingsNoiseMaxSet(uint8_t *NewNoiseMax);
extern void SystemIdentSettingsNoiseMaxGet(uint8_t *NewNoiseMax);
extern void SystemIdentSettingsCalculateYawSet(SystemIdentSettingsCalculateYawOptions *NewCalculateYaw);
extern void SystemIdentSettingsCalculateYawGet(SystemIdentSettingsCalculateYawOptions *NewCalculateYaw);
extern void SystemIdentSettingsDestinationPidBankSet(SystemIdentSettingsDestinationPidBankOptions *NewDestinationPidBank);
extern void SystemIdentSettingsDestinationPidBankGet(SystemIdentSettingsDestinationPidBankOptions *NewDestinationPidBank);
extern void SystemIdentSettingsTuningDurationSet(uint8_t *NewTuningDuration);
extern void SystemIdentSettingsTuningDurationGet(uint8_t *NewTuningDuration);
extern void SystemIdentSettingsThrustControlSet(SystemIdentSettingsThrustControlOptions *NewThrustControl);
extern void SystemIdentSettingsThrustControlGet(SystemIdentSettingsThrustControlOptions *NewThrustControl);
extern void SystemIdentSettingsSmoothQuickSourceSet(SystemIdentSettingsSmoothQuickSourceOptions *NewSmoothQuickSource);
extern void SystemIdentSettingsSmoothQuickSourceGet(SystemIdentSettingsSmoothQuickSourceOptions *NewSmoothQuickSource);
extern void SystemIdentSettingsDisableSanityChecksSet(SystemIdentSettingsDisableSanityChecksOptions *NewDisableSanityChecks);
extern void SystemIdentSettingsDisableSanityChecksGet(SystemIdentSettingsDisableSanityChecksOptions *NewDisableSanityChecks);
extern void SystemIdentSettingsCompleteSet(SystemIdentSettingsCompleteOptions *NewComplete);
extern void SystemIdentSettingsCompleteGet(SystemIdentSettingsCompleteOptions *NewComplete);


#endif // SYSTEMIDENTSETTINGS_H

/**
 * @}
 * @}
 */
