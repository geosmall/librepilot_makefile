/**
 ******************************************************************************
 * @addtogroup UAVObjects OpenPilot UAVObjects
 * @{
 * @addtogroup HwPikoBLXSettings HwPikoBLXSettings
 * @brief Furious FPV Piko BLX Micro Flight Controller hardware configuration
 *
 * Autogenerated files and functions for HwPikoBLXSettings Object
 *
 * @{
 *
 * @file       hwpikoblxsettings.h
 * @author     The OpenPilot Team, http://www.openpilot.org Copyright (C) 2010-2013.
 * @brief      Implementation of the HwPikoBLXSettings object. This file has been
 *             automatically generated by the UAVObjectGenerator.
 *
 * @note       Object definition file: hwpikoblxsettings.xml.
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

#ifndef HWPIKOBLXSETTINGS_H
#define HWPIKOBLXSETTINGS_H
#include <stdbool.h>

/* Object constants */
#define HWPIKOBLXSETTINGS_OBJID 0xBE5F8824
#define HWPIKOBLXSETTINGS_ISSINGLEINST 1
#define HWPIKOBLXSETTINGS_ISSETTINGS 1
#define HWPIKOBLXSETTINGS_ISPRIORITY 0
#define HWPIKOBLXSETTINGS_NUMBYTES sizeof(HwPikoBLXSettingsData)

/* Generic interface functions */
int32_t HwPikoBLXSettingsInitialize();
UAVObjHandle HwPikoBLXSettingsHandle();
void HwPikoBLXSettingsSetDefaults(UAVObjHandle obj, uint16_t instId);

/* Field UARTPort information */

// Enumeration options for field UARTPort
typedef enum __attribute__ ((__packed__)) {
    HWPIKOBLXSETTINGS_UARTPORT_DISABLED=0,
    HWPIKOBLXSETTINGS_UARTPORT_TELEMETRY=1,
    HWPIKOBLXSETTINGS_UARTPORT_GPS=2,
    HWPIKOBLXSETTINGS_UARTPORT_SBUS=3,
    HWPIKOBLXSETTINGS_UARTPORT_DSM=4,
    HWPIKOBLXSETTINGS_UARTPORT_EXBUS=5,
    HWPIKOBLXSETTINGS_UARTPORT_HOTTSUMD=6,
    HWPIKOBLXSETTINGS_UARTPORT_HOTTSUMH=7,
    HWPIKOBLXSETTINGS_UARTPORT_SRXL=8,
    HWPIKOBLXSETTINGS_UARTPORT_IBUS=9,
    HWPIKOBLXSETTINGS_UARTPORT_DEBUGCONSOLE=10,
    HWPIKOBLXSETTINGS_UARTPORT_COMBRIDGE=11,
    HWPIKOBLXSETTINGS_UARTPORT_MSP=12,
    HWPIKOBLXSETTINGS_UARTPORT_MAVLINK=13,
    HWPIKOBLXSETTINGS_UARTPORT_HOTTTELEMETRY=14,
    HWPIKOBLXSETTINGS_UARTPORT_FRSKYSENSORHUB=15
} HwPikoBLXSettingsUARTPortOptions;

// Number of elements for field UARTPort
#define HWPIKOBLXSETTINGS_UARTPORT_NUMELEM 3

/* Field LEDPort information */

// Enumeration options for field LEDPort
typedef enum __attribute__ ((__packed__)) {
    HWPIKOBLXSETTINGS_LEDPORT_DISABLED=0,
    HWPIKOBLXSETTINGS_LEDPORT_WS281X=1
} HwPikoBLXSettingsLEDPortOptions;

/* Field PPMPort information */

// Enumeration options for field PPMPort
typedef enum __attribute__ ((__packed__)) {
    HWPIKOBLXSETTINGS_PPMPORT_DISABLED=0,
    HWPIKOBLXSETTINGS_PPMPORT_ENABLED=1
} HwPikoBLXSettingsPPMPortOptions;




/*
 * Packed Object data (unaligned).
 * Should only be used where 4 byte alignment can be guaranteed
 * (eg a single instance on the heap)
 */
typedef struct {
        HwPikoBLXSettingsUARTPortOptions UARTPort[3];
    HwPikoBLXSettingsLEDPortOptions LEDPort;
    HwPikoBLXSettingsPPMPortOptions PPMPort;

} __attribute__((packed)) HwPikoBLXSettingsDataPacked;

/*
 * Packed Object data.
 * Alignment is forced to 4 bytes so as to avoid the potential for CPU usage faults
 * on Cortex M4F during load/store of float UAVO fields
 */
typedef HwPikoBLXSettingsDataPacked __attribute__((aligned(4))) HwPikoBLXSettingsData;

void HwPikoBLXSettingsDataOverrideDefaults(HwPikoBLXSettingsData * data);

/* Typesafe Object access functions */
static inline int32_t HwPikoBLXSettingsGet(HwPikoBLXSettingsData * dataOut) {
    return UAVObjGetData(HwPikoBLXSettingsHandle(), dataOut);
}
static inline int32_t HwPikoBLXSettingsSet(const HwPikoBLXSettingsData * dataIn) {
    return UAVObjSetData(HwPikoBLXSettingsHandle(), dataIn);
}
static inline int32_t HwPikoBLXSettingsInstGet(uint16_t instId, HwPikoBLXSettingsData * dataOut) {
    return UAVObjGetInstanceData(HwPikoBLXSettingsHandle(), instId, dataOut);
}
static inline int32_t HwPikoBLXSettingsInstSet(uint16_t instId, const HwPikoBLXSettingsData * dataIn) {
    return UAVObjSetInstanceData(HwPikoBLXSettingsHandle(), instId, dataIn);
}
static inline int32_t HwPikoBLXSettingsConnectQueue(xQueueHandle queue) {
    return UAVObjConnectQueue(HwPikoBLXSettingsHandle(), queue, EV_MASK_ALL_UPDATES);
}
static inline int32_t HwPikoBLXSettingsConnectCallback(UAVObjEventCallback cb) {
    return UAVObjConnectCallback(HwPikoBLXSettingsHandle(), cb, EV_MASK_ALL_UPDATES, false);
}
static inline int32_t HwPikoBLXSettingsConnectFastCallback(UAVObjEventCallback cb) {
    return UAVObjConnectCallback(HwPikoBLXSettingsHandle(), cb, EV_MASK_ALL_UPDATES, true);
}
static inline uint16_t HwPikoBLXSettingsCreateInstance() {
    return UAVObjCreateInstance(HwPikoBLXSettingsHandle());
}
static inline void HwPikoBLXSettingsRequestUpdate() {
    UAVObjRequestUpdate(HwPikoBLXSettingsHandle());
}
static inline void HwPikoBLXSettingsRequestInstUpdate(uint16_t instId) {
    UAVObjRequestInstanceUpdate(HwPikoBLXSettingsHandle(), instId);
}
static inline void HwPikoBLXSettingsUpdated() {
    UAVObjUpdated(HwPikoBLXSettingsHandle());
}
static inline void HwPikoBLXSettingsInstUpdated(uint16_t instId) {
    UAVObjInstanceUpdated(HwPikoBLXSettingsHandle(), instId);
}
static inline void HwPikoBLXSettingsLogging() {
    UAVObjLogging(HwPikoBLXSettingsHandle());
}
static inline void HwPikoBLXSettingsInstLogging(uint16_t instId) {
    UAVObjInstanceLogging(HwPikoBLXSettingsHandle(), instId);
}
static inline int32_t HwPikoBLXSettingsGetMetadata(UAVObjMetadata * dataOut) {
    return UAVObjGetMetadata(HwPikoBLXSettingsHandle(), dataOut);
}
static inline int32_t HwPikoBLXSettingsSetMetadata(const UAVObjMetadata * dataIn) {
    return UAVObjSetMetadata(HwPikoBLXSettingsHandle(), dataIn);
}
static inline int8_t HwPikoBLXSettingsReadOnly() {
    return UAVObjReadOnly(HwPikoBLXSettingsHandle());
}

/* Set/Get functions */
extern void HwPikoBLXSettingsUARTPortSet(HwPikoBLXSettingsUARTPortOptions *NewUARTPort);
extern void HwPikoBLXSettingsUARTPortGet(HwPikoBLXSettingsUARTPortOptions *NewUARTPort);
extern void HwPikoBLXSettingsLEDPortSet(HwPikoBLXSettingsLEDPortOptions *NewLEDPort);
extern void HwPikoBLXSettingsLEDPortGet(HwPikoBLXSettingsLEDPortOptions *NewLEDPort);
extern void HwPikoBLXSettingsPPMPortSet(HwPikoBLXSettingsPPMPortOptions *NewPPMPort);
extern void HwPikoBLXSettingsPPMPortGet(HwPikoBLXSettingsPPMPortOptions *NewPPMPort);


#endif // HWPIKOBLXSETTINGS_H

/**
 * @}
 * @}
 */
