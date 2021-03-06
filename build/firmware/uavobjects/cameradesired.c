/**
 ******************************************************************************
 * @addtogroup UAVObjects OpenPilot UAVObjects
 * @{ 
 * @addtogroup CameraDesired CameraDesired
 * @brief Desired camera outputs.  Comes from @ref CameraStabilization module.
 *
 * Autogenerated files and functions for CameraDesired Object
 * @{ 
 *
 * @file       cameradesired.c
 * @author     The OpenPilot Team, http://www.openpilot.org Copyright (C) 2010-2013.
 * @brief      Implementation of the CameraDesired object. This file has been 
 *             automatically generated by the UAVObjectGenerator.
 * 
 * @note       Object definition file: cameradesired.xml. 
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
#include "cameradesired.h"

// Private variables
#if (defined(__MACH__) && defined(__APPLE__))
static UAVObjHandle handle __attribute__((section("__DATA,_uavo_handles")));
#else
static UAVObjHandle handle __attribute__((section("_uavo_handles")));
#endif

#if CAMERADESIRED_ISSETTINGS
SETTINGS_INITCALL(CameraDesiredInitialize);
#endif

/**
 * Initialize object.
 * \return 0 Success
 * \return -1 Failure to initialize or -2 for already initialized
 */
int32_t CameraDesiredInitialize(void)
{
    // Compile time assertion that the CameraDesiredDataPacked and CameraDesiredData structs
    // have the same size (though instances of CameraDesiredData
    // should be placed in memory by the linker/compiler on a 4 byte alignment).
    PIOS_STATIC_ASSERT(sizeof(CameraDesiredDataPacked) == sizeof(CameraDesiredData));
    
    // Don't set the handle to null if already registered
    if (UAVObjGetByID(CAMERADESIRED_OBJID)) {
        return -2;
    }

    static const UAVObjType objType = {
       .id = CAMERADESIRED_OBJID,
       .instance_size = CAMERADESIRED_NUMBYTES,
       .init_callback = &CameraDesiredSetDefaults,
    };

    // Register object with the object manager
    handle = UAVObjRegister(&objType,
        CAMERADESIRED_ISSINGLEINST, CAMERADESIRED_ISSETTINGS, CAMERADESIRED_ISPRIORITY);

    // Done
    return handle ? 0 : -1;
}

static inline void DataOverrideDefaults(__attribute__((unused)) CameraDesiredData * data) {}

void CameraDesiredDataOverrideDefaults(CameraDesiredData * data) __attribute__((weak, alias("DataOverrideDefaults")));

/**
 * Initialize object fields and metadata with the default values.
 * If a default value is not specified the object fields
 * will be initialized to zero.
 */
void CameraDesiredSetDefaults(UAVObjHandle obj, uint16_t instId)
{
    CameraDesiredData data;

    // Initialize object fields to their default values
    UAVObjGetInstanceData(obj, instId, &data);
    memset(&data, 0, sizeof(CameraDesiredData));

    CameraDesiredDataOverrideDefaults(&data);
    UAVObjSetInstanceData(obj, instId, &data);

    // Initialize object metadata to their default values
    if ( instId == 0 ) {
        UAVObjMetadata metadata;
        metadata.flags =
            ACCESS_READWRITE << UAVOBJ_ACCESS_SHIFT |
            ACCESS_READWRITE << UAVOBJ_GCS_ACCESS_SHIFT |
            0 << UAVOBJ_TELEMETRY_ACKED_SHIFT |
            0 << UAVOBJ_GCS_TELEMETRY_ACKED_SHIFT |
            UPDATEMODE_THROTTLED << UAVOBJ_TELEMETRY_UPDATE_MODE_SHIFT |
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
UAVObjHandle CameraDesiredHandle()
{
    return handle;
}

/**
 * Get/Set object Functions
 */
void CameraDesiredRollOrServo1Set(float *NewRollOrServo1)
{
    UAVObjSetDataField(CameraDesiredHandle(), (void *)NewRollOrServo1, offsetof(CameraDesiredData, RollOrServo1), sizeof(float));
}
void CameraDesiredRollOrServo1Get(float *NewRollOrServo1)
{
    UAVObjGetDataField(CameraDesiredHandle(), (void *)NewRollOrServo1, offsetof(CameraDesiredData, RollOrServo1), sizeof(float));
}
void CameraDesiredPitchOrServo2Set(float *NewPitchOrServo2)
{
    UAVObjSetDataField(CameraDesiredHandle(), (void *)NewPitchOrServo2, offsetof(CameraDesiredData, PitchOrServo2), sizeof(float));
}
void CameraDesiredPitchOrServo2Get(float *NewPitchOrServo2)
{
    UAVObjGetDataField(CameraDesiredHandle(), (void *)NewPitchOrServo2, offsetof(CameraDesiredData, PitchOrServo2), sizeof(float));
}
void CameraDesiredYawSet(float *NewYaw)
{
    UAVObjSetDataField(CameraDesiredHandle(), (void *)NewYaw, offsetof(CameraDesiredData, Yaw), sizeof(float));
}
void CameraDesiredYawGet(float *NewYaw)
{
    UAVObjGetDataField(CameraDesiredHandle(), (void *)NewYaw, offsetof(CameraDesiredData, Yaw), sizeof(float));
}
void CameraDesiredTriggerSet(float *NewTrigger)
{
    UAVObjSetDataField(CameraDesiredHandle(), (void *)NewTrigger, offsetof(CameraDesiredData, Trigger), sizeof(float));
}
void CameraDesiredTriggerGet(float *NewTrigger)
{
    UAVObjGetDataField(CameraDesiredHandle(), (void *)NewTrigger, offsetof(CameraDesiredData, Trigger), sizeof(float));
}


/**
 * @}
 */
