/**
 ******************************************************************************
 * @addtogroup UAVObjects OpenPilot UAVObjects
 * @{
 * @addtogroup StabilizationSettingsBank1 StabilizationSettingsBank1
 * @brief Currently selected PID bank
 *
 * Autogenerated files and functions for StabilizationSettingsBank1 Object
 *
 * @{
 *
 * @file       stabilizationsettingsbank1.h
 * @author     The OpenPilot Team, http://www.openpilot.org Copyright (C) 2010-2013.
 * @brief      Implementation of the StabilizationSettingsBank1 object. This file has been
 *             automatically generated by the UAVObjectGenerator.
 *
 * @note       Object definition file: stabilizationsettingsbank1.xml.
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

#ifndef STABILIZATIONSETTINGSBANK1_H
#define STABILIZATIONSETTINGSBANK1_H
#include <stdbool.h>

/* Object constants */
#define STABILIZATIONSETTINGSBANK1_OBJID 0xACB1E81C
#define STABILIZATIONSETTINGSBANK1_ISSINGLEINST 1
#define STABILIZATIONSETTINGSBANK1_ISSETTINGS 1
#define STABILIZATIONSETTINGSBANK1_ISPRIORITY 0
#define STABILIZATIONSETTINGSBANK1_NUMBYTES sizeof(StabilizationSettingsBank1Data)

/* Generic interface functions */
int32_t StabilizationSettingsBank1Initialize();
UAVObjHandle StabilizationSettingsBank1Handle();
void StabilizationSettingsBank1SetDefaults(UAVObjHandle obj, uint16_t instId);

/* Field AttitudeFeedForward information */

// Array element names for field AttitudeFeedForward
typedef enum {
    STABILIZATIONSETTINGSBANK1_ATTITUDEFEEDFORWARD_ROLL=0,
    STABILIZATIONSETTINGSBANK1_ATTITUDEFEEDFORWARD_PITCH=1,
    STABILIZATIONSETTINGSBANK1_ATTITUDEFEEDFORWARD_YAW=2
} StabilizationSettingsBank1AttitudeFeedForwardElem;

// Number of elements for field AttitudeFeedForward
#define STABILIZATIONSETTINGSBANK1_ATTITUDEFEEDFORWARD_NUMELEM 3

/* Field RollRatePID information */

// Array element names for field RollRatePID
typedef enum {
    STABILIZATIONSETTINGSBANK1_ROLLRATEPID_KP=0,
    STABILIZATIONSETTINGSBANK1_ROLLRATEPID_KI=1,
    STABILIZATIONSETTINGSBANK1_ROLLRATEPID_KD=2,
    STABILIZATIONSETTINGSBANK1_ROLLRATEPID_ILIMIT=3
} StabilizationSettingsBank1RollRatePIDElem;

// Number of elements for field RollRatePID
#define STABILIZATIONSETTINGSBANK1_ROLLRATEPID_NUMELEM 4

/* Field PitchRatePID information */

// Array element names for field PitchRatePID
typedef enum {
    STABILIZATIONSETTINGSBANK1_PITCHRATEPID_KP=0,
    STABILIZATIONSETTINGSBANK1_PITCHRATEPID_KI=1,
    STABILIZATIONSETTINGSBANK1_PITCHRATEPID_KD=2,
    STABILIZATIONSETTINGSBANK1_PITCHRATEPID_ILIMIT=3
} StabilizationSettingsBank1PitchRatePIDElem;

// Number of elements for field PitchRatePID
#define STABILIZATIONSETTINGSBANK1_PITCHRATEPID_NUMELEM 4

/* Field YawRatePID information */

// Array element names for field YawRatePID
typedef enum {
    STABILIZATIONSETTINGSBANK1_YAWRATEPID_KP=0,
    STABILIZATIONSETTINGSBANK1_YAWRATEPID_KI=1,
    STABILIZATIONSETTINGSBANK1_YAWRATEPID_KD=2,
    STABILIZATIONSETTINGSBANK1_YAWRATEPID_ILIMIT=3
} StabilizationSettingsBank1YawRatePIDElem;

// Number of elements for field YawRatePID
#define STABILIZATIONSETTINGSBANK1_YAWRATEPID_NUMELEM 4

/* Field RollPI information */

// Array element names for field RollPI
typedef enum {
    STABILIZATIONSETTINGSBANK1_ROLLPI_KP=0,
    STABILIZATIONSETTINGSBANK1_ROLLPI_KI=1,
    STABILIZATIONSETTINGSBANK1_ROLLPI_ILIMIT=2
} StabilizationSettingsBank1RollPIElem;

// Number of elements for field RollPI
#define STABILIZATIONSETTINGSBANK1_ROLLPI_NUMELEM 3

/* Field PitchPI information */

// Array element names for field PitchPI
typedef enum {
    STABILIZATIONSETTINGSBANK1_PITCHPI_KP=0,
    STABILIZATIONSETTINGSBANK1_PITCHPI_KI=1,
    STABILIZATIONSETTINGSBANK1_PITCHPI_ILIMIT=2
} StabilizationSettingsBank1PitchPIElem;

// Number of elements for field PitchPI
#define STABILIZATIONSETTINGSBANK1_PITCHPI_NUMELEM 3

/* Field YawPI information */

// Array element names for field YawPI
typedef enum {
    STABILIZATIONSETTINGSBANK1_YAWPI_KP=0,
    STABILIZATIONSETTINGSBANK1_YAWPI_KI=1,
    STABILIZATIONSETTINGSBANK1_YAWPI_ILIMIT=2
} StabilizationSettingsBank1YawPIElem;

// Number of elements for field YawPI
#define STABILIZATIONSETTINGSBANK1_YAWPI_NUMELEM 3

/* Field ManualRate information */

// Array element names for field ManualRate
typedef enum {
    STABILIZATIONSETTINGSBANK1_MANUALRATE_ROLL=0,
    STABILIZATIONSETTINGSBANK1_MANUALRATE_PITCH=1,
    STABILIZATIONSETTINGSBANK1_MANUALRATE_YAW=2
} StabilizationSettingsBank1ManualRateElem;

// Number of elements for field ManualRate
#define STABILIZATIONSETTINGSBANK1_MANUALRATE_NUMELEM 3

/* Field MaximumRate information */

// Array element names for field MaximumRate
typedef enum {
    STABILIZATIONSETTINGSBANK1_MAXIMUMRATE_ROLL=0,
    STABILIZATIONSETTINGSBANK1_MAXIMUMRATE_PITCH=1,
    STABILIZATIONSETTINGSBANK1_MAXIMUMRATE_YAW=2
} StabilizationSettingsBank1MaximumRateElem;

// Number of elements for field MaximumRate
#define STABILIZATIONSETTINGSBANK1_MAXIMUMRATE_NUMELEM 3

/* Field RollMax information */

/* Field PitchMax information */

/* Field YawMax information */

/* Field StickExpo information */

// Array element names for field StickExpo
typedef enum {
    STABILIZATIONSETTINGSBANK1_STICKEXPO_ROLL=0,
    STABILIZATIONSETTINGSBANK1_STICKEXPO_PITCH=1,
    STABILIZATIONSETTINGSBANK1_STICKEXPO_YAW=2
} StabilizationSettingsBank1StickExpoElem;

// Number of elements for field StickExpo
#define STABILIZATIONSETTINGSBANK1_STICKEXPO_NUMELEM 3

/* Field AcroInsanityFactor information */

// Array element names for field AcroInsanityFactor
typedef enum {
    STABILIZATIONSETTINGSBANK1_ACROINSANITYFACTOR_ROLL=0,
    STABILIZATIONSETTINGSBANK1_ACROINSANITYFACTOR_PITCH=1,
    STABILIZATIONSETTINGSBANK1_ACROINSANITYFACTOR_YAW=2
} StabilizationSettingsBank1AcroInsanityFactorElem;

// Number of elements for field AcroInsanityFactor
#define STABILIZATIONSETTINGSBANK1_ACROINSANITYFACTOR_NUMELEM 3

/* Field EnablePiroComp information */

// Enumeration options for field EnablePiroComp
typedef enum __attribute__ ((__packed__)) {
    STABILIZATIONSETTINGSBANK1_ENABLEPIROCOMP_FALSE=0,
    STABILIZATIONSETTINGSBANK1_ENABLEPIROCOMP_TRUE=1
} StabilizationSettingsBank1EnablePiroCompOptions;

/* Field FpvCamTiltCompensation information */

/* Field EnableThrustPIDScaling information */

// Enumeration options for field EnableThrustPIDScaling
typedef enum __attribute__ ((__packed__)) {
    STABILIZATIONSETTINGSBANK1_ENABLETHRUSTPIDSCALING_FALSE=0,
    STABILIZATIONSETTINGSBANK1_ENABLETHRUSTPIDSCALING_TRUE=1
} StabilizationSettingsBank1EnableThrustPIDScalingOptions;

/* Field ThrustPIDScaleCurve information */

// Array element names for field ThrustPIDScaleCurve
typedef enum {
    STABILIZATIONSETTINGSBANK1_THRUSTPIDSCALECURVE_0=0,
    STABILIZATIONSETTINGSBANK1_THRUSTPIDSCALECURVE_25=1,
    STABILIZATIONSETTINGSBANK1_THRUSTPIDSCALECURVE_50=2,
    STABILIZATIONSETTINGSBANK1_THRUSTPIDSCALECURVE_75=3,
    STABILIZATIONSETTINGSBANK1_THRUSTPIDSCALECURVE_100=4
} StabilizationSettingsBank1ThrustPIDScaleCurveElem;

// Number of elements for field ThrustPIDScaleCurve
#define STABILIZATIONSETTINGSBANK1_THRUSTPIDSCALECURVE_NUMELEM 5

/* Field ThrustPIDScaleSource information */

// Enumeration options for field ThrustPIDScaleSource
typedef enum __attribute__ ((__packed__)) {
    STABILIZATIONSETTINGSBANK1_THRUSTPIDSCALESOURCE_MANUALCONTROLTHROTTLE=0,
    STABILIZATIONSETTINGSBANK1_THRUSTPIDSCALESOURCE_STABILIZATIONDESIREDTHRUST=1,
    STABILIZATIONSETTINGSBANK1_THRUSTPIDSCALESOURCE_ACTUATORDESIREDTHRUST=2
} StabilizationSettingsBank1ThrustPIDScaleSourceOptions;

/* Field ThrustPIDScaleTarget information */

// Enumeration options for field ThrustPIDScaleTarget
typedef enum __attribute__ ((__packed__)) {
    STABILIZATIONSETTINGSBANK1_THRUSTPIDSCALETARGET_PID=0,
    STABILIZATIONSETTINGSBANK1_THRUSTPIDSCALETARGET_PI=1,
    STABILIZATIONSETTINGSBANK1_THRUSTPIDSCALETARGET_PD=2,
    STABILIZATIONSETTINGSBANK1_THRUSTPIDSCALETARGET_ID=3,
    STABILIZATIONSETTINGSBANK1_THRUSTPIDSCALETARGET_P=4,
    STABILIZATIONSETTINGSBANK1_THRUSTPIDSCALETARGET_I=5,
    STABILIZATIONSETTINGSBANK1_THRUSTPIDSCALETARGET_D=6
} StabilizationSettingsBank1ThrustPIDScaleTargetOptions;

/* Field ThrustPIDScaleAxes information */

// Enumeration options for field ThrustPIDScaleAxes
typedef enum __attribute__ ((__packed__)) {
    STABILIZATIONSETTINGSBANK1_THRUSTPIDSCALEAXES_ROLLPITCHYAW=0,
    STABILIZATIONSETTINGSBANK1_THRUSTPIDSCALEAXES_ROLLPITCH=1,
    STABILIZATIONSETTINGSBANK1_THRUSTPIDSCALEAXES_ROLLYAW=2,
    STABILIZATIONSETTINGSBANK1_THRUSTPIDSCALEAXES_ROLL=3,
    STABILIZATIONSETTINGSBANK1_THRUSTPIDSCALEAXES_PITCHYAW=4,
    STABILIZATIONSETTINGSBANK1_THRUSTPIDSCALEAXES_PITCH=5,
    STABILIZATIONSETTINGSBANK1_THRUSTPIDSCALEAXES_YAW=6
} StabilizationSettingsBank1ThrustPIDScaleAxesOptions;



typedef struct __attribute__ ((__packed__)) {
    float Roll;
    float Pitch;
    float Yaw;
}  StabilizationSettingsBank1AttitudeFeedForwardData ;
typedef struct __attribute__ ((__packed__)) {
    float array[3];
}  StabilizationSettingsBank1AttitudeFeedForwardDataArray ;
#define StabilizationSettingsBank1AttitudeFeedForwardToArray( var ) UAVObjectFieldToArray( StabilizationSettingsBank1AttitudeFeedForwardData, var )

typedef struct __attribute__ ((__packed__)) {
    float Kp;
    float Ki;
    float Kd;
    float ILimit;
}  StabilizationSettingsBank1RollRatePIDData ;
typedef struct __attribute__ ((__packed__)) {
    float array[4];
}  StabilizationSettingsBank1RollRatePIDDataArray ;
#define StabilizationSettingsBank1RollRatePIDToArray( var ) UAVObjectFieldToArray( StabilizationSettingsBank1RollRatePIDData, var )

typedef struct __attribute__ ((__packed__)) {
    float Kp;
    float Ki;
    float Kd;
    float ILimit;
}  StabilizationSettingsBank1PitchRatePIDData ;
typedef struct __attribute__ ((__packed__)) {
    float array[4];
}  StabilizationSettingsBank1PitchRatePIDDataArray ;
#define StabilizationSettingsBank1PitchRatePIDToArray( var ) UAVObjectFieldToArray( StabilizationSettingsBank1PitchRatePIDData, var )

typedef struct __attribute__ ((__packed__)) {
    float Kp;
    float Ki;
    float Kd;
    float ILimit;
}  StabilizationSettingsBank1YawRatePIDData ;
typedef struct __attribute__ ((__packed__)) {
    float array[4];
}  StabilizationSettingsBank1YawRatePIDDataArray ;
#define StabilizationSettingsBank1YawRatePIDToArray( var ) UAVObjectFieldToArray( StabilizationSettingsBank1YawRatePIDData, var )

typedef struct __attribute__ ((__packed__)) {
    float Kp;
    float Ki;
    float ILimit;
}  StabilizationSettingsBank1RollPIData ;
typedef struct __attribute__ ((__packed__)) {
    float array[3];
}  StabilizationSettingsBank1RollPIDataArray ;
#define StabilizationSettingsBank1RollPIToArray( var ) UAVObjectFieldToArray( StabilizationSettingsBank1RollPIData, var )

typedef struct __attribute__ ((__packed__)) {
    float Kp;
    float Ki;
    float ILimit;
}  StabilizationSettingsBank1PitchPIData ;
typedef struct __attribute__ ((__packed__)) {
    float array[3];
}  StabilizationSettingsBank1PitchPIDataArray ;
#define StabilizationSettingsBank1PitchPIToArray( var ) UAVObjectFieldToArray( StabilizationSettingsBank1PitchPIData, var )

typedef struct __attribute__ ((__packed__)) {
    float Kp;
    float Ki;
    float ILimit;
}  StabilizationSettingsBank1YawPIData ;
typedef struct __attribute__ ((__packed__)) {
    float array[3];
}  StabilizationSettingsBank1YawPIDataArray ;
#define StabilizationSettingsBank1YawPIToArray( var ) UAVObjectFieldToArray( StabilizationSettingsBank1YawPIData, var )

typedef struct __attribute__ ((__packed__)) {
    uint16_t Roll;
    uint16_t Pitch;
    uint16_t Yaw;
}  StabilizationSettingsBank1ManualRateData ;
typedef struct __attribute__ ((__packed__)) {
    uint16_t array[3];
}  StabilizationSettingsBank1ManualRateDataArray ;
#define StabilizationSettingsBank1ManualRateToArray( var ) UAVObjectFieldToArray( StabilizationSettingsBank1ManualRateData, var )

typedef struct __attribute__ ((__packed__)) {
    uint16_t Roll;
    uint16_t Pitch;
    uint16_t Yaw;
}  StabilizationSettingsBank1MaximumRateData ;
typedef struct __attribute__ ((__packed__)) {
    uint16_t array[3];
}  StabilizationSettingsBank1MaximumRateDataArray ;
#define StabilizationSettingsBank1MaximumRateToArray( var ) UAVObjectFieldToArray( StabilizationSettingsBank1MaximumRateData, var )

typedef struct __attribute__ ((__packed__)) {
    int8_t Roll;
    int8_t Pitch;
    int8_t Yaw;
}  StabilizationSettingsBank1StickExpoData ;
typedef struct __attribute__ ((__packed__)) {
    int8_t array[3];
}  StabilizationSettingsBank1StickExpoDataArray ;
#define StabilizationSettingsBank1StickExpoToArray( var ) UAVObjectFieldToArray( StabilizationSettingsBank1StickExpoData, var )

typedef struct __attribute__ ((__packed__)) {
    uint8_t Roll;
    uint8_t Pitch;
    uint8_t Yaw;
}  StabilizationSettingsBank1AcroInsanityFactorData ;
typedef struct __attribute__ ((__packed__)) {
    uint8_t array[3];
}  StabilizationSettingsBank1AcroInsanityFactorDataArray ;
#define StabilizationSettingsBank1AcroInsanityFactorToArray( var ) UAVObjectFieldToArray( StabilizationSettingsBank1AcroInsanityFactorData, var )


/*
 * Packed Object data (unaligned).
 * Should only be used where 4 byte alignment can be guaranteed
 * (eg a single instance on the heap)
 */
typedef struct {
        StabilizationSettingsBank1AttitudeFeedForwardData AttitudeFeedForward;
    StabilizationSettingsBank1RollRatePIDData RollRatePID;
    StabilizationSettingsBank1PitchRatePIDData PitchRatePID;
    StabilizationSettingsBank1YawRatePIDData YawRatePID;
    StabilizationSettingsBank1RollPIData RollPI;
    StabilizationSettingsBank1PitchPIData PitchPI;
    StabilizationSettingsBank1YawPIData YawPI;
    StabilizationSettingsBank1ManualRateData ManualRate;
    StabilizationSettingsBank1MaximumRateData MaximumRate;
    uint8_t RollMax;
    uint8_t PitchMax;
    uint8_t YawMax;
    StabilizationSettingsBank1StickExpoData StickExpo;
    StabilizationSettingsBank1AcroInsanityFactorData AcroInsanityFactor;
    StabilizationSettingsBank1EnablePiroCompOptions EnablePiroComp;
    uint8_t FpvCamTiltCompensation;
    StabilizationSettingsBank1EnableThrustPIDScalingOptions EnableThrustPIDScaling;
    int8_t ThrustPIDScaleCurve[5];
    StabilizationSettingsBank1ThrustPIDScaleSourceOptions ThrustPIDScaleSource;
    StabilizationSettingsBank1ThrustPIDScaleTargetOptions ThrustPIDScaleTarget;
    StabilizationSettingsBank1ThrustPIDScaleAxesOptions ThrustPIDScaleAxes;

} __attribute__((packed)) StabilizationSettingsBank1DataPacked;

/*
 * Packed Object data.
 * Alignment is forced to 4 bytes so as to avoid the potential for CPU usage faults
 * on Cortex M4F during load/store of float UAVO fields
 */
typedef StabilizationSettingsBank1DataPacked __attribute__((aligned(4))) StabilizationSettingsBank1Data;

void StabilizationSettingsBank1DataOverrideDefaults(StabilizationSettingsBank1Data * data);

/* Typesafe Object access functions */
static inline int32_t StabilizationSettingsBank1Get(StabilizationSettingsBank1Data * dataOut) {
    return UAVObjGetData(StabilizationSettingsBank1Handle(), dataOut);
}
static inline int32_t StabilizationSettingsBank1Set(const StabilizationSettingsBank1Data * dataIn) {
    return UAVObjSetData(StabilizationSettingsBank1Handle(), dataIn);
}
static inline int32_t StabilizationSettingsBank1InstGet(uint16_t instId, StabilizationSettingsBank1Data * dataOut) {
    return UAVObjGetInstanceData(StabilizationSettingsBank1Handle(), instId, dataOut);
}
static inline int32_t StabilizationSettingsBank1InstSet(uint16_t instId, const StabilizationSettingsBank1Data * dataIn) {
    return UAVObjSetInstanceData(StabilizationSettingsBank1Handle(), instId, dataIn);
}
static inline int32_t StabilizationSettingsBank1ConnectQueue(xQueueHandle queue) {
    return UAVObjConnectQueue(StabilizationSettingsBank1Handle(), queue, EV_MASK_ALL_UPDATES);
}
static inline int32_t StabilizationSettingsBank1ConnectCallback(UAVObjEventCallback cb) {
    return UAVObjConnectCallback(StabilizationSettingsBank1Handle(), cb, EV_MASK_ALL_UPDATES, false);
}
static inline int32_t StabilizationSettingsBank1ConnectFastCallback(UAVObjEventCallback cb) {
    return UAVObjConnectCallback(StabilizationSettingsBank1Handle(), cb, EV_MASK_ALL_UPDATES, true);
}
static inline uint16_t StabilizationSettingsBank1CreateInstance() {
    return UAVObjCreateInstance(StabilizationSettingsBank1Handle());
}
static inline void StabilizationSettingsBank1RequestUpdate() {
    UAVObjRequestUpdate(StabilizationSettingsBank1Handle());
}
static inline void StabilizationSettingsBank1RequestInstUpdate(uint16_t instId) {
    UAVObjRequestInstanceUpdate(StabilizationSettingsBank1Handle(), instId);
}
static inline void StabilizationSettingsBank1Updated() {
    UAVObjUpdated(StabilizationSettingsBank1Handle());
}
static inline void StabilizationSettingsBank1InstUpdated(uint16_t instId) {
    UAVObjInstanceUpdated(StabilizationSettingsBank1Handle(), instId);
}
static inline void StabilizationSettingsBank1Logging() {
    UAVObjLogging(StabilizationSettingsBank1Handle());
}
static inline void StabilizationSettingsBank1InstLogging(uint16_t instId) {
    UAVObjInstanceLogging(StabilizationSettingsBank1Handle(), instId);
}
static inline int32_t StabilizationSettingsBank1GetMetadata(UAVObjMetadata * dataOut) {
    return UAVObjGetMetadata(StabilizationSettingsBank1Handle(), dataOut);
}
static inline int32_t StabilizationSettingsBank1SetMetadata(const UAVObjMetadata * dataIn) {
    return UAVObjSetMetadata(StabilizationSettingsBank1Handle(), dataIn);
}
static inline int8_t StabilizationSettingsBank1ReadOnly() {
    return UAVObjReadOnly(StabilizationSettingsBank1Handle());
}

/* Set/Get functions */
extern void StabilizationSettingsBank1AttitudeFeedForwardSet(StabilizationSettingsBank1AttitudeFeedForwardData *NewAttitudeFeedForward);
extern void StabilizationSettingsBank1AttitudeFeedForwardGet(StabilizationSettingsBank1AttitudeFeedForwardData *NewAttitudeFeedForward);
extern void StabilizationSettingsBank1AttitudeFeedForwardArraySet(float *NewAttitudeFeedForward);
extern void StabilizationSettingsBank1AttitudeFeedForwardArrayGet(float *NewAttitudeFeedForward);
extern void StabilizationSettingsBank1RollRatePIDSet(StabilizationSettingsBank1RollRatePIDData *NewRollRatePID);
extern void StabilizationSettingsBank1RollRatePIDGet(StabilizationSettingsBank1RollRatePIDData *NewRollRatePID);
extern void StabilizationSettingsBank1RollRatePIDArraySet(float *NewRollRatePID);
extern void StabilizationSettingsBank1RollRatePIDArrayGet(float *NewRollRatePID);
extern void StabilizationSettingsBank1PitchRatePIDSet(StabilizationSettingsBank1PitchRatePIDData *NewPitchRatePID);
extern void StabilizationSettingsBank1PitchRatePIDGet(StabilizationSettingsBank1PitchRatePIDData *NewPitchRatePID);
extern void StabilizationSettingsBank1PitchRatePIDArraySet(float *NewPitchRatePID);
extern void StabilizationSettingsBank1PitchRatePIDArrayGet(float *NewPitchRatePID);
extern void StabilizationSettingsBank1YawRatePIDSet(StabilizationSettingsBank1YawRatePIDData *NewYawRatePID);
extern void StabilizationSettingsBank1YawRatePIDGet(StabilizationSettingsBank1YawRatePIDData *NewYawRatePID);
extern void StabilizationSettingsBank1YawRatePIDArraySet(float *NewYawRatePID);
extern void StabilizationSettingsBank1YawRatePIDArrayGet(float *NewYawRatePID);
extern void StabilizationSettingsBank1RollPISet(StabilizationSettingsBank1RollPIData *NewRollPI);
extern void StabilizationSettingsBank1RollPIGet(StabilizationSettingsBank1RollPIData *NewRollPI);
extern void StabilizationSettingsBank1RollPIArraySet(float *NewRollPI);
extern void StabilizationSettingsBank1RollPIArrayGet(float *NewRollPI);
extern void StabilizationSettingsBank1PitchPISet(StabilizationSettingsBank1PitchPIData *NewPitchPI);
extern void StabilizationSettingsBank1PitchPIGet(StabilizationSettingsBank1PitchPIData *NewPitchPI);
extern void StabilizationSettingsBank1PitchPIArraySet(float *NewPitchPI);
extern void StabilizationSettingsBank1PitchPIArrayGet(float *NewPitchPI);
extern void StabilizationSettingsBank1YawPISet(StabilizationSettingsBank1YawPIData *NewYawPI);
extern void StabilizationSettingsBank1YawPIGet(StabilizationSettingsBank1YawPIData *NewYawPI);
extern void StabilizationSettingsBank1YawPIArraySet(float *NewYawPI);
extern void StabilizationSettingsBank1YawPIArrayGet(float *NewYawPI);
extern void StabilizationSettingsBank1ManualRateSet(StabilizationSettingsBank1ManualRateData *NewManualRate);
extern void StabilizationSettingsBank1ManualRateGet(StabilizationSettingsBank1ManualRateData *NewManualRate);
extern void StabilizationSettingsBank1ManualRateArraySet(uint16_t *NewManualRate);
extern void StabilizationSettingsBank1ManualRateArrayGet(uint16_t *NewManualRate);
extern void StabilizationSettingsBank1MaximumRateSet(StabilizationSettingsBank1MaximumRateData *NewMaximumRate);
extern void StabilizationSettingsBank1MaximumRateGet(StabilizationSettingsBank1MaximumRateData *NewMaximumRate);
extern void StabilizationSettingsBank1MaximumRateArraySet(uint16_t *NewMaximumRate);
extern void StabilizationSettingsBank1MaximumRateArrayGet(uint16_t *NewMaximumRate);
extern void StabilizationSettingsBank1RollMaxSet(uint8_t *NewRollMax);
extern void StabilizationSettingsBank1RollMaxGet(uint8_t *NewRollMax);
extern void StabilizationSettingsBank1PitchMaxSet(uint8_t *NewPitchMax);
extern void StabilizationSettingsBank1PitchMaxGet(uint8_t *NewPitchMax);
extern void StabilizationSettingsBank1YawMaxSet(uint8_t *NewYawMax);
extern void StabilizationSettingsBank1YawMaxGet(uint8_t *NewYawMax);
extern void StabilizationSettingsBank1StickExpoSet(StabilizationSettingsBank1StickExpoData *NewStickExpo);
extern void StabilizationSettingsBank1StickExpoGet(StabilizationSettingsBank1StickExpoData *NewStickExpo);
extern void StabilizationSettingsBank1StickExpoArraySet(int8_t *NewStickExpo);
extern void StabilizationSettingsBank1StickExpoArrayGet(int8_t *NewStickExpo);
extern void StabilizationSettingsBank1AcroInsanityFactorSet(StabilizationSettingsBank1AcroInsanityFactorData *NewAcroInsanityFactor);
extern void StabilizationSettingsBank1AcroInsanityFactorGet(StabilizationSettingsBank1AcroInsanityFactorData *NewAcroInsanityFactor);
extern void StabilizationSettingsBank1AcroInsanityFactorArraySet(uint8_t *NewAcroInsanityFactor);
extern void StabilizationSettingsBank1AcroInsanityFactorArrayGet(uint8_t *NewAcroInsanityFactor);
extern void StabilizationSettingsBank1EnablePiroCompSet(StabilizationSettingsBank1EnablePiroCompOptions *NewEnablePiroComp);
extern void StabilizationSettingsBank1EnablePiroCompGet(StabilizationSettingsBank1EnablePiroCompOptions *NewEnablePiroComp);
extern void StabilizationSettingsBank1FpvCamTiltCompensationSet(uint8_t *NewFpvCamTiltCompensation);
extern void StabilizationSettingsBank1FpvCamTiltCompensationGet(uint8_t *NewFpvCamTiltCompensation);
extern void StabilizationSettingsBank1EnableThrustPIDScalingSet(StabilizationSettingsBank1EnableThrustPIDScalingOptions *NewEnableThrustPIDScaling);
extern void StabilizationSettingsBank1EnableThrustPIDScalingGet(StabilizationSettingsBank1EnableThrustPIDScalingOptions *NewEnableThrustPIDScaling);
extern void StabilizationSettingsBank1ThrustPIDScaleCurveSet(int8_t *NewThrustPIDScaleCurve);
extern void StabilizationSettingsBank1ThrustPIDScaleCurveGet(int8_t *NewThrustPIDScaleCurve);
extern void StabilizationSettingsBank1ThrustPIDScaleSourceSet(StabilizationSettingsBank1ThrustPIDScaleSourceOptions *NewThrustPIDScaleSource);
extern void StabilizationSettingsBank1ThrustPIDScaleSourceGet(StabilizationSettingsBank1ThrustPIDScaleSourceOptions *NewThrustPIDScaleSource);
extern void StabilizationSettingsBank1ThrustPIDScaleTargetSet(StabilizationSettingsBank1ThrustPIDScaleTargetOptions *NewThrustPIDScaleTarget);
extern void StabilizationSettingsBank1ThrustPIDScaleTargetGet(StabilizationSettingsBank1ThrustPIDScaleTargetOptions *NewThrustPIDScaleTarget);
extern void StabilizationSettingsBank1ThrustPIDScaleAxesSet(StabilizationSettingsBank1ThrustPIDScaleAxesOptions *NewThrustPIDScaleAxes);
extern void StabilizationSettingsBank1ThrustPIDScaleAxesGet(StabilizationSettingsBank1ThrustPIDScaleAxesOptions *NewThrustPIDScaleAxes);


#endif // STABILIZATIONSETTINGSBANK1_H

/**
 * @}
 * @}
 */
