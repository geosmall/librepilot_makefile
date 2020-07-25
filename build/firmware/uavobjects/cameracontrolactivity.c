/**
 ******************************************************************************
 * @addtogroup UAVObjects OpenPilot UAVObjects
 * @{ 
 * @addtogroup CameraControlActivity CameraControlActivity
 * @brief Contains position and timestamp of each camera operation
 *
 * Autogenerated files and functions for CameraControlActivity Object
 * @{ 
 *
 * @file       cameracontrolactivity.c
 * @author     The OpenPilot Team, http://www.openpilot.org Copyright (C) 2010-2013.
 * @brief      Implementation of the CameraControlActivity object. This file has been 
 *             automatically generated by the UAVObjectGenerator.
 * 
 * @note       Object definition file: cameracontrolactivity.xml. 
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

#include <openpilot.h>
#include "cameracontrolactivity.h"

// Private variables
#if (defined(__MACH__) && defined(__APPLE__))
static UAVObjHandle handle __attribute__((section("__DATA,_uavo_handles")));
#else
static UAVObjHandle handle __attribute__((section("_uavo_handles")));
#endif

#if CAMERACONTROLACTIVITY_ISSETTINGS
SETTINGS_INITCALL(CameraControlActivityInitialize);
#endif

/**
 * Initialize object.
 * \return 0 Success
 * \return -1 Failure to initialize or -2 for already initialized
 */
int32_t CameraControlActivityInitialize(void)
{
    // Compile time assertion that the CameraControlActivityDataPacked and CameraControlActivityData structs
    // have the same size (though instances of CameraControlActivityData
    // should be placed in memory by the linker/compiler on a 4 byte alignment).
    PIOS_STATIC_ASSERT(sizeof(CameraControlActivityDataPacked) == sizeof(CameraControlActivityData));
    
    // Don't set the handle to null if already registered
    if (UAVObjGetByID(CAMERACONTROLACTIVITY_OBJID)) {
        return -2;
    }

    static const UAVObjType objType = {
       .id = CAMERACONTROLACTIVITY_OBJID,
       .instance_size = CAMERACONTROLACTIVITY_NUMBYTES,
       .init_callback = &CameraControlActivitySetDefaults,
    };

    // Register object with the object manager
    handle = UAVObjRegister(&objType,
        CAMERACONTROLACTIVITY_ISSINGLEINST, CAMERACONTROLACTIVITY_ISSETTINGS, CAMERACONTROLACTIVITY_ISPRIORITY);

    // Done
    return handle ? 0 : -1;
}

static inline void DataOverrideDefaults(__attribute__((unused)) CameraControlActivityData * data) {}

void CameraControlActivityDataOverrideDefaults(CameraControlActivityData * data) __attribute__((weak, alias("DataOverrideDefaults")));

/**
 * Initialize object fields and metadata with the default values.
 * If a default value is not specified the object fields
 * will be initialized to zero.
 */
void CameraControlActivitySetDefaults(UAVObjHandle obj, uint16_t instId)
{
    CameraControlActivityData data;

    // Initialize object fields to their default values
    UAVObjGetInstanceData(obj, instId, &data);
    memset(&data, 0, sizeof(CameraControlActivityData));

    CameraControlActivityDataOverrideDefaults(&data);
    UAVObjSetInstanceData(obj, instId, &data);

    // Initialize object metadata to their default values
    if ( instId == 0 ) {
        UAVObjMetadata metadata;
        metadata.flags =
            ACCESS_READWRITE << UAVOBJ_ACCESS_SHIFT |
            ACCESS_READWRITE << UAVOBJ_GCS_ACCESS_SHIFT |
            0 << UAVOBJ_TELEMETRY_ACKED_SHIFT |
            0 << UAVOBJ_GCS_TELEMETRY_ACKED_SHIFT |
            UPDATEMODE_PERIODIC << UAVOBJ_TELEMETRY_UPDATE_MODE_SHIFT |
            UPDATEMODE_MANUAL << UAVOBJ_GCS_TELEMETRY_UPDATE_MODE_SHIFT |
            UPDATEMODE_MANUAL << UAVOBJ_LOGGING_UPDATE_MODE_SHIFT;
        metadata.telemetryUpdatePeriod = 1000;
        metadata.gcsTelemetryUpdatePeriod = 0;
        metadata.loggingUpdatePeriod = 0;
        UAVObjSetMetadata(obj, &metadata);
    }
}

/**
 * Get object handle
 */
UAVObjHandle CameraControlActivityHandle()
{
    return handle;
}

/**
 * Get/Set object Functions
 */
void CameraControlActivityLatitudeSet(int32_t *NewLatitude)
{
    UAVObjSetDataField(CameraControlActivityHandle(), (void *)NewLatitude, offsetof(CameraControlActivityData, Latitude), sizeof(int32_t));
}
void CameraControlActivityLatitudeGet(int32_t *NewLatitude)
{
    UAVObjGetDataField(CameraControlActivityHandle(), (void *)NewLatitude, offsetof(CameraControlActivityData, Latitude), sizeof(int32_t));
}
void CameraControlActivityLongitudeSet(int32_t *NewLongitude)
{
    UAVObjSetDataField(CameraControlActivityHandle(), (void *)NewLongitude, offsetof(CameraControlActivityData, Longitude), sizeof(int32_t));
}
void CameraControlActivityLongitudeGet(int32_t *NewLongitude)
{
    UAVObjGetDataField(CameraControlActivityHandle(), (void *)NewLongitude, offsetof(CameraControlActivityData, Longitude), sizeof(int32_t));
}
void CameraControlActivityAltitudeSet(float *NewAltitude)
{
    UAVObjSetDataField(CameraControlActivityHandle(), (void *)NewAltitude, offsetof(CameraControlActivityData, Altitude), sizeof(float));
}
void CameraControlActivityAltitudeGet(float *NewAltitude)
{
    UAVObjGetDataField(CameraControlActivityHandle(), (void *)NewAltitude, offsetof(CameraControlActivityData, Altitude), sizeof(float));
}
void CameraControlActivityRollSet(float *NewRoll)
{
    UAVObjSetDataField(CameraControlActivityHandle(), (void *)NewRoll, offsetof(CameraControlActivityData, Roll), sizeof(float));
}
void CameraControlActivityRollGet(float *NewRoll)
{
    UAVObjGetDataField(CameraControlActivityHandle(), (void *)NewRoll, offsetof(CameraControlActivityData, Roll), sizeof(float));
}
void CameraControlActivityPitchSet(float *NewPitch)
{
    UAVObjSetDataField(CameraControlActivityHandle(), (void *)NewPitch, offsetof(CameraControlActivityData, Pitch), sizeof(float));
}
void CameraControlActivityPitchGet(float *NewPitch)
{
    UAVObjGetDataField(CameraControlActivityHandle(), (void *)NewPitch, offsetof(CameraControlActivityData, Pitch), sizeof(float));
}
void CameraControlActivityYawSet(float *NewYaw)
{
    UAVObjSetDataField(CameraControlActivityHandle(), (void *)NewYaw, offsetof(CameraControlActivityData, Yaw), sizeof(float));
}
void CameraControlActivityYawGet(float *NewYaw)
{
    UAVObjGetDataField(CameraControlActivityHandle(), (void *)NewYaw, offsetof(CameraControlActivityData, Yaw), sizeof(float));
}
void CameraControlActivitySystemTSSet(uint32_t *NewSystemTS)
{
    UAVObjSetDataField(CameraControlActivityHandle(), (void *)NewSystemTS, offsetof(CameraControlActivityData, SystemTS), sizeof(uint32_t));
}
void CameraControlActivitySystemTSGet(uint32_t *NewSystemTS)
{
    UAVObjGetDataField(CameraControlActivityHandle(), (void *)NewSystemTS, offsetof(CameraControlActivityData, SystemTS), sizeof(uint32_t));
}
void CameraControlActivityImageIdSet(uint16_t *NewImageId)
{
    UAVObjSetDataField(CameraControlActivityHandle(), (void *)NewImageId, offsetof(CameraControlActivityData, ImageId), sizeof(uint16_t));
}
void CameraControlActivityImageIdGet(uint16_t *NewImageId)
{
    UAVObjGetDataField(CameraControlActivityHandle(), (void *)NewImageId, offsetof(CameraControlActivityData, ImageId), sizeof(uint16_t));
}
void CameraControlActivityTriggerYearSet(int16_t *NewTriggerYear)
{
    UAVObjSetDataField(CameraControlActivityHandle(), (void *)NewTriggerYear, offsetof(CameraControlActivityData, TriggerYear), sizeof(int16_t));
}
void CameraControlActivityTriggerYearGet(int16_t *NewTriggerYear)
{
    UAVObjGetDataField(CameraControlActivityHandle(), (void *)NewTriggerYear, offsetof(CameraControlActivityData, TriggerYear), sizeof(int16_t));
}
void CameraControlActivityTriggerMillisecondSet(int16_t *NewTriggerMillisecond)
{
    UAVObjSetDataField(CameraControlActivityHandle(), (void *)NewTriggerMillisecond, offsetof(CameraControlActivityData, TriggerMillisecond), sizeof(int16_t));
}
void CameraControlActivityTriggerMillisecondGet(int16_t *NewTriggerMillisecond)
{
    UAVObjGetDataField(CameraControlActivityHandle(), (void *)NewTriggerMillisecond, offsetof(CameraControlActivityData, TriggerMillisecond), sizeof(int16_t));
}
void CameraControlActivityTriggerMonthSet(int8_t *NewTriggerMonth)
{
    UAVObjSetDataField(CameraControlActivityHandle(), (void *)NewTriggerMonth, offsetof(CameraControlActivityData, TriggerMonth), sizeof(int8_t));
}
void CameraControlActivityTriggerMonthGet(int8_t *NewTriggerMonth)
{
    UAVObjGetDataField(CameraControlActivityHandle(), (void *)NewTriggerMonth, offsetof(CameraControlActivityData, TriggerMonth), sizeof(int8_t));
}
void CameraControlActivityTriggerDaySet(int8_t *NewTriggerDay)
{
    UAVObjSetDataField(CameraControlActivityHandle(), (void *)NewTriggerDay, offsetof(CameraControlActivityData, TriggerDay), sizeof(int8_t));
}
void CameraControlActivityTriggerDayGet(int8_t *NewTriggerDay)
{
    UAVObjGetDataField(CameraControlActivityHandle(), (void *)NewTriggerDay, offsetof(CameraControlActivityData, TriggerDay), sizeof(int8_t));
}
void CameraControlActivityTriggerHourSet(int8_t *NewTriggerHour)
{
    UAVObjSetDataField(CameraControlActivityHandle(), (void *)NewTriggerHour, offsetof(CameraControlActivityData, TriggerHour), sizeof(int8_t));
}
void CameraControlActivityTriggerHourGet(int8_t *NewTriggerHour)
{
    UAVObjGetDataField(CameraControlActivityHandle(), (void *)NewTriggerHour, offsetof(CameraControlActivityData, TriggerHour), sizeof(int8_t));
}
void CameraControlActivityTriggerMinuteSet(int8_t *NewTriggerMinute)
{
    UAVObjSetDataField(CameraControlActivityHandle(), (void *)NewTriggerMinute, offsetof(CameraControlActivityData, TriggerMinute), sizeof(int8_t));
}
void CameraControlActivityTriggerMinuteGet(int8_t *NewTriggerMinute)
{
    UAVObjGetDataField(CameraControlActivityHandle(), (void *)NewTriggerMinute, offsetof(CameraControlActivityData, TriggerMinute), sizeof(int8_t));
}
void CameraControlActivityTriggerSecondSet(int8_t *NewTriggerSecond)
{
    UAVObjSetDataField(CameraControlActivityHandle(), (void *)NewTriggerSecond, offsetof(CameraControlActivityData, TriggerSecond), sizeof(int8_t));
}
void CameraControlActivityTriggerSecondGet(int8_t *NewTriggerSecond)
{
    UAVObjGetDataField(CameraControlActivityHandle(), (void *)NewTriggerSecond, offsetof(CameraControlActivityData, TriggerSecond), sizeof(int8_t));
}
void CameraControlActivityActivitySet(CameraControlActivityActivityOptions *NewActivity)
{
    UAVObjSetDataField(CameraControlActivityHandle(), (void *)NewActivity, offsetof(CameraControlActivityData, Activity), sizeof(CameraControlActivityActivityOptions));
}
void CameraControlActivityActivityGet(CameraControlActivityActivityOptions *NewActivity)
{
    UAVObjGetDataField(CameraControlActivityHandle(), (void *)NewActivity, offsetof(CameraControlActivityData, Activity), sizeof(CameraControlActivityActivityOptions));
}
void CameraControlActivityReasonSet(CameraControlActivityReasonOptions *NewReason)
{
    UAVObjSetDataField(CameraControlActivityHandle(), (void *)NewReason, offsetof(CameraControlActivityData, Reason), sizeof(CameraControlActivityReasonOptions));
}
void CameraControlActivityReasonGet(CameraControlActivityReasonOptions *NewReason)
{
    UAVObjGetDataField(CameraControlActivityHandle(), (void *)NewReason, offsetof(CameraControlActivityData, Reason), sizeof(CameraControlActivityReasonOptions));
}


/**
 * @}
 */
