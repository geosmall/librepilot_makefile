/**
 ******************************************************************************
 * @addtogroup UAVObjects OpenPilot UAVObjects
 * @{ 
 * @addtogroup CallbackInfo CallbackInfo
 * @brief Task information
 *
 * Autogenerated files and functions for CallbackInfo Object
 * @{ 
 *
 * @file       callbackinfo.c
 * @author     The OpenPilot Team, http://www.openpilot.org Copyright (C) 2010-2013.
 * @brief      Implementation of the CallbackInfo object. This file has been 
 *             automatically generated by the UAVObjectGenerator.
 * 
 * @note       Object definition file: callbackinfo.xml. 
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
#include "callbackinfo.h"

// Private variables
#if (defined(__MACH__) && defined(__APPLE__))
static UAVObjHandle handle __attribute__((section("__DATA,_uavo_handles")));
#else
static UAVObjHandle handle __attribute__((section("_uavo_handles")));
#endif

#if CALLBACKINFO_ISSETTINGS
SETTINGS_INITCALL(CallbackInfoInitialize);
#endif

/**
 * Initialize object.
 * \return 0 Success
 * \return -1 Failure to initialize or -2 for already initialized
 */
int32_t CallbackInfoInitialize(void)
{
    // Compile time assertion that the CallbackInfoDataPacked and CallbackInfoData structs
    // have the same size (though instances of CallbackInfoData
    // should be placed in memory by the linker/compiler on a 4 byte alignment).
    PIOS_STATIC_ASSERT(sizeof(CallbackInfoDataPacked) == sizeof(CallbackInfoData));
    
    // Don't set the handle to null if already registered
    if (UAVObjGetByID(CALLBACKINFO_OBJID)) {
        return -2;
    }

    static const UAVObjType objType = {
       .id = CALLBACKINFO_OBJID,
       .instance_size = CALLBACKINFO_NUMBYTES,
       .init_callback = &CallbackInfoSetDefaults,
    };

    // Register object with the object manager
    handle = UAVObjRegister(&objType,
        CALLBACKINFO_ISSINGLEINST, CALLBACKINFO_ISSETTINGS, CALLBACKINFO_ISPRIORITY);

    // Done
    return handle ? 0 : -1;
}

static inline void DataOverrideDefaults(__attribute__((unused)) CallbackInfoData * data) {}

void CallbackInfoDataOverrideDefaults(CallbackInfoData * data) __attribute__((weak, alias("DataOverrideDefaults")));

/**
 * Initialize object fields and metadata with the default values.
 * If a default value is not specified the object fields
 * will be initialized to zero.
 */
void CallbackInfoSetDefaults(UAVObjHandle obj, uint16_t instId)
{
    CallbackInfoData data;

    // Initialize object fields to their default values
    UAVObjGetInstanceData(obj, instId, &data);
    memset(&data, 0, sizeof(CallbackInfoData));

    CallbackInfoDataOverrideDefaults(&data);
    UAVObjSetInstanceData(obj, instId, &data);

    // Initialize object metadata to their default values
    if ( instId == 0 ) {
        UAVObjMetadata metadata;
        metadata.flags =
            ACCESS_READWRITE << UAVOBJ_ACCESS_SHIFT |
            ACCESS_READONLY << UAVOBJ_GCS_ACCESS_SHIFT |
            0 << UAVOBJ_TELEMETRY_ACKED_SHIFT |
            0 << UAVOBJ_GCS_TELEMETRY_ACKED_SHIFT |
            UPDATEMODE_PERIODIC << UAVOBJ_TELEMETRY_UPDATE_MODE_SHIFT |
            UPDATEMODE_ONCHANGE << UAVOBJ_GCS_TELEMETRY_UPDATE_MODE_SHIFT |
            UPDATEMODE_MANUAL << UAVOBJ_LOGGING_UPDATE_MODE_SHIFT;
        metadata.telemetryUpdatePeriod = 10000;
        metadata.gcsTelemetryUpdatePeriod = 0;
        metadata.loggingUpdatePeriod = 0;
        UAVObjSetMetadata(obj, &metadata);
    }
}

/**
 * Get object handle
 */
UAVObjHandle CallbackInfoHandle()
{
    return handle;
}

/**
 * Get/Set object Functions
 */
void CallbackInfoRunningTimeSet( CallbackInfoRunningTimeData *NewRunningTime )
{
    UAVObjSetDataField(CallbackInfoHandle(), (void *)NewRunningTime, offsetof(CallbackInfoData, RunningTime), 11*sizeof(uint32_t));
}
void CallbackInfoRunningTimeGet( CallbackInfoRunningTimeData *NewRunningTime )
{
    UAVObjGetDataField(CallbackInfoHandle(), (void *)NewRunningTime, offsetof(CallbackInfoData, RunningTime), 11*sizeof(uint32_t));
}
void CallbackInfoRunningTimeArraySet( uint32_t *NewRunningTime )
{
    UAVObjSetDataField(CallbackInfoHandle(), (void *)NewRunningTime, offsetof(CallbackInfoData, RunningTime), 11*sizeof(uint32_t));
}
void CallbackInfoRunningTimeArrayGet( uint32_t *NewRunningTime )
{
    UAVObjGetDataField(CallbackInfoHandle(), (void *)NewRunningTime, offsetof(CallbackInfoData, RunningTime), 11*sizeof(uint32_t));
}
void CallbackInfoStackRemainingSet( CallbackInfoStackRemainingData *NewStackRemaining )
{
    UAVObjSetDataField(CallbackInfoHandle(), (void *)NewStackRemaining, offsetof(CallbackInfoData, StackRemaining), 11*sizeof(int16_t));
}
void CallbackInfoStackRemainingGet( CallbackInfoStackRemainingData *NewStackRemaining )
{
    UAVObjGetDataField(CallbackInfoHandle(), (void *)NewStackRemaining, offsetof(CallbackInfoData, StackRemaining), 11*sizeof(int16_t));
}
void CallbackInfoStackRemainingArraySet( int16_t *NewStackRemaining )
{
    UAVObjSetDataField(CallbackInfoHandle(), (void *)NewStackRemaining, offsetof(CallbackInfoData, StackRemaining), 11*sizeof(int16_t));
}
void CallbackInfoStackRemainingArrayGet( int16_t *NewStackRemaining )
{
    UAVObjGetDataField(CallbackInfoHandle(), (void *)NewStackRemaining, offsetof(CallbackInfoData, StackRemaining), 11*sizeof(int16_t));
}
void CallbackInfoRunningSet( CallbackInfoRunningData *NewRunning )
{
    UAVObjSetDataField(CallbackInfoHandle(), (void *)NewRunning, offsetof(CallbackInfoData, Running), 11*sizeof(CallbackInfoRunningOptions));
}
void CallbackInfoRunningGet( CallbackInfoRunningData *NewRunning )
{
    UAVObjGetDataField(CallbackInfoHandle(), (void *)NewRunning, offsetof(CallbackInfoData, Running), 11*sizeof(CallbackInfoRunningOptions));
}
void CallbackInfoRunningArraySet( CallbackInfoRunningOptions *NewRunning )
{
    UAVObjSetDataField(CallbackInfoHandle(), (void *)NewRunning, offsetof(CallbackInfoData, Running), 11*sizeof(CallbackInfoRunningOptions));
}
void CallbackInfoRunningArrayGet( CallbackInfoRunningOptions *NewRunning )
{
    UAVObjGetDataField(CallbackInfoHandle(), (void *)NewRunning, offsetof(CallbackInfoData, Running), 11*sizeof(CallbackInfoRunningOptions));
}


/**
 * @}
 */
