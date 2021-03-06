/**
 ******************************************************************************
 * @addtogroup UAVObjects OpenPilot UAVObjects
 * @{
 * @addtogroup HwDiscoveryF4BareSettings HwDiscoveryF4BareSettings
 * @brief DiscoveryF4 Bare hardware configuration
 *
 * Autogenerated files and functions for HwDiscoveryF4BareSettings Object
 *
 * @{
 *
 * @file       hwdiscoveryf4baresettings.h
 * @author     The OpenPilot Team, http://www.openpilot.org Copyright (C) 2010-2013.
 * @brief      Implementation of the HwDiscoveryF4BareSettings object. This file has been
 *             automatically generated by the UAVObjectGenerator.
 *
 * @note       Object definition file: hwdiscoveryf4baresettings.xml.
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

#ifndef HWDISCOVERYF4BARESETTINGS_H
#define HWDISCOVERYF4BARESETTINGS_H
#include <stdbool.h>

/* Object constants */
#define HWDISCOVERYF4BARESETTINGS_OBJID 0xA95D2DFE
#define HWDISCOVERYF4BARESETTINGS_ISSINGLEINST 1
#define HWDISCOVERYF4BARESETTINGS_ISSETTINGS 1
#define HWDISCOVERYF4BARESETTINGS_ISPRIORITY 0
#define HWDISCOVERYF4BARESETTINGS_NUMBYTES sizeof(HwDiscoveryF4BareSettingsData)

/* Generic interface functions */
int32_t HwDiscoveryF4BareSettingsInitialize();
UAVObjHandle HwDiscoveryF4BareSettingsHandle();
void HwDiscoveryF4BareSettingsSetDefaults(UAVObjHandle obj, uint16_t instId);

/* Field BoardRevision information */

/* Field BoardType information */




/*
 * Packed Object data (unaligned).
 * Should only be used where 4 byte alignment can be guaranteed
 * (eg a single instance on the heap)
 */
typedef struct {
        uint16_t BoardRevision;
    uint8_t BoardType;

} __attribute__((packed)) HwDiscoveryF4BareSettingsDataPacked;

/*
 * Packed Object data.
 * Alignment is forced to 4 bytes so as to avoid the potential for CPU usage faults
 * on Cortex M4F during load/store of float UAVO fields
 */
typedef HwDiscoveryF4BareSettingsDataPacked __attribute__((aligned(4))) HwDiscoveryF4BareSettingsData;

void HwDiscoveryF4BareSettingsDataOverrideDefaults(HwDiscoveryF4BareSettingsData * data);

/* Typesafe Object access functions */
static inline int32_t HwDiscoveryF4BareSettingsGet(HwDiscoveryF4BareSettingsData * dataOut) {
    return UAVObjGetData(HwDiscoveryF4BareSettingsHandle(), dataOut);
}
static inline int32_t HwDiscoveryF4BareSettingsSet(const HwDiscoveryF4BareSettingsData * dataIn) {
    return UAVObjSetData(HwDiscoveryF4BareSettingsHandle(), dataIn);
}
static inline int32_t HwDiscoveryF4BareSettingsInstGet(uint16_t instId, HwDiscoveryF4BareSettingsData * dataOut) {
    return UAVObjGetInstanceData(HwDiscoveryF4BareSettingsHandle(), instId, dataOut);
}
static inline int32_t HwDiscoveryF4BareSettingsInstSet(uint16_t instId, const HwDiscoveryF4BareSettingsData * dataIn) {
    return UAVObjSetInstanceData(HwDiscoveryF4BareSettingsHandle(), instId, dataIn);
}
static inline int32_t HwDiscoveryF4BareSettingsConnectQueue(xQueueHandle queue) {
    return UAVObjConnectQueue(HwDiscoveryF4BareSettingsHandle(), queue, EV_MASK_ALL_UPDATES);
}
static inline int32_t HwDiscoveryF4BareSettingsConnectCallback(UAVObjEventCallback cb) {
    return UAVObjConnectCallback(HwDiscoveryF4BareSettingsHandle(), cb, EV_MASK_ALL_UPDATES, false);
}
static inline int32_t HwDiscoveryF4BareSettingsConnectFastCallback(UAVObjEventCallback cb) {
    return UAVObjConnectCallback(HwDiscoveryF4BareSettingsHandle(), cb, EV_MASK_ALL_UPDATES, true);
}
static inline uint16_t HwDiscoveryF4BareSettingsCreateInstance() {
    return UAVObjCreateInstance(HwDiscoveryF4BareSettingsHandle());
}
static inline void HwDiscoveryF4BareSettingsRequestUpdate() {
    UAVObjRequestUpdate(HwDiscoveryF4BareSettingsHandle());
}
static inline void HwDiscoveryF4BareSettingsRequestInstUpdate(uint16_t instId) {
    UAVObjRequestInstanceUpdate(HwDiscoveryF4BareSettingsHandle(), instId);
}
static inline void HwDiscoveryF4BareSettingsUpdated() {
    UAVObjUpdated(HwDiscoveryF4BareSettingsHandle());
}
static inline void HwDiscoveryF4BareSettingsInstUpdated(uint16_t instId) {
    UAVObjInstanceUpdated(HwDiscoveryF4BareSettingsHandle(), instId);
}
static inline void HwDiscoveryF4BareSettingsLogging() {
    UAVObjLogging(HwDiscoveryF4BareSettingsHandle());
}
static inline void HwDiscoveryF4BareSettingsInstLogging(uint16_t instId) {
    UAVObjInstanceLogging(HwDiscoveryF4BareSettingsHandle(), instId);
}
static inline int32_t HwDiscoveryF4BareSettingsGetMetadata(UAVObjMetadata * dataOut) {
    return UAVObjGetMetadata(HwDiscoveryF4BareSettingsHandle(), dataOut);
}
static inline int32_t HwDiscoveryF4BareSettingsSetMetadata(const UAVObjMetadata * dataIn) {
    return UAVObjSetMetadata(HwDiscoveryF4BareSettingsHandle(), dataIn);
}
static inline int8_t HwDiscoveryF4BareSettingsReadOnly() {
    return UAVObjReadOnly(HwDiscoveryF4BareSettingsHandle());
}

/* Set/Get functions */
extern void HwDiscoveryF4BareSettingsBoardRevisionSet(uint16_t *NewBoardRevision);
extern void HwDiscoveryF4BareSettingsBoardRevisionGet(uint16_t *NewBoardRevision);
extern void HwDiscoveryF4BareSettingsBoardTypeSet(uint8_t *NewBoardType);
extern void HwDiscoveryF4BareSettingsBoardTypeGet(uint8_t *NewBoardType);


#endif // HWDISCOVERYF4BARESETTINGS_H

/**
 * @}
 * @}
 */
