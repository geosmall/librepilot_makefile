/**
 ******************************************************************************
 * @addtogroup UAVObjects OpenPilot UAVObjects
 * @{ 
 * @addtogroup AccelSensor AccelSensor
 * @brief Calibrated sensor data from 3 axis accelerometer in m/s².
 *
 * Autogenerated files and functions for AccelSensor Object
 * @{ 
 *
 * @file       accelsensor.c
 * @author     The OpenPilot Team, http://www.openpilot.org Copyright (C) 2010-2013.
 * @brief      Implementation of the AccelSensor object. This file has been 
 *             automatically generated by the UAVObjectGenerator.
 * 
 * @note       Object definition file: accelsensor.xml. 
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
#include "accelsensor.h"

// Private variables
#if (defined(__MACH__) && defined(__APPLE__))
static UAVObjHandle handle __attribute__((section("__DATA,_uavo_handles")));
#else
static UAVObjHandle handle __attribute__((section("_uavo_handles")));
#endif

#if ACCELSENSOR_ISSETTINGS
SETTINGS_INITCALL(AccelSensorInitialize);
#endif

/**
 * Initialize object.
 * \return 0 Success
 * \return -1 Failure to initialize or -2 for already initialized
 */
int32_t AccelSensorInitialize(void)
{
    // Compile time assertion that the AccelSensorDataPacked and AccelSensorData structs
    // have the same size (though instances of AccelSensorData
    // should be placed in memory by the linker/compiler on a 4 byte alignment).
    PIOS_STATIC_ASSERT(sizeof(AccelSensorDataPacked) == sizeof(AccelSensorData));
    
    // Don't set the handle to null if already registered
    if (UAVObjGetByID(ACCELSENSOR_OBJID)) {
        return -2;
    }

    static const UAVObjType objType = {
       .id = ACCELSENSOR_OBJID,
       .instance_size = ACCELSENSOR_NUMBYTES,
       .init_callback = &AccelSensorSetDefaults,
    };

    // Register object with the object manager
    handle = UAVObjRegister(&objType,
        ACCELSENSOR_ISSINGLEINST, ACCELSENSOR_ISSETTINGS, ACCELSENSOR_ISPRIORITY);

    // Done
    return handle ? 0 : -1;
}

static inline void DataOverrideDefaults(__attribute__((unused)) AccelSensorData * data) {}

void AccelSensorDataOverrideDefaults(AccelSensorData * data) __attribute__((weak, alias("DataOverrideDefaults")));

/**
 * Initialize object fields and metadata with the default values.
 * If a default value is not specified the object fields
 * will be initialized to zero.
 */
void AccelSensorSetDefaults(UAVObjHandle obj, uint16_t instId)
{
    AccelSensorData data;

    // Initialize object fields to their default values
    UAVObjGetInstanceData(obj, instId, &data);
    memset(&data, 0, sizeof(AccelSensorData));

    AccelSensorDataOverrideDefaults(&data);
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
        metadata.telemetryUpdatePeriod = 10000;
        metadata.gcsTelemetryUpdatePeriod = 0;
        metadata.loggingUpdatePeriod = 0;
        UAVObjSetMetadata(obj, &metadata);
    }
}

/**
 * Get object handle
 */
UAVObjHandle AccelSensorHandle()
{
    return handle;
}

/**
 * Get/Set object Functions
 */
void AccelSensorxSet(float *Newx)
{
    UAVObjSetDataField(AccelSensorHandle(), (void *)Newx, offsetof(AccelSensorData, x), sizeof(float));
}
void AccelSensorxGet(float *Newx)
{
    UAVObjGetDataField(AccelSensorHandle(), (void *)Newx, offsetof(AccelSensorData, x), sizeof(float));
}
void AccelSensorySet(float *Newy)
{
    UAVObjSetDataField(AccelSensorHandle(), (void *)Newy, offsetof(AccelSensorData, y), sizeof(float));
}
void AccelSensoryGet(float *Newy)
{
    UAVObjGetDataField(AccelSensorHandle(), (void *)Newy, offsetof(AccelSensorData, y), sizeof(float));
}
void AccelSensorzSet(float *Newz)
{
    UAVObjSetDataField(AccelSensorHandle(), (void *)Newz, offsetof(AccelSensorData, z), sizeof(float));
}
void AccelSensorzGet(float *Newz)
{
    UAVObjGetDataField(AccelSensorHandle(), (void *)Newz, offsetof(AccelSensorData, z), sizeof(float));
}
void AccelSensortemperatureSet(float *Newtemperature)
{
    UAVObjSetDataField(AccelSensorHandle(), (void *)Newtemperature, offsetof(AccelSensorData, temperature), sizeof(float));
}
void AccelSensortemperatureGet(float *Newtemperature)
{
    UAVObjGetDataField(AccelSensorHandle(), (void *)Newtemperature, offsetof(AccelSensorData, temperature), sizeof(float));
}


/**
 * @}
 */
