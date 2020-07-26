CC=arm-none-eabi-gcc
CXX=arm-none-eabi-g++
AR=arm-none-eabi-ar
OBJCOPY=arm-none-eabi-objcopy

###########################################

vpath %.c flight
vpath %.cpp flight


DEFINES  = -DUSE_INPUT_LPF -DUSE_GIMBAL_LPF -DUSE_GIMBAL_FF -DDIAG_TASKS -DSTM32F4XX -DSTM32F4 -DPIOS_TARGET_PROVIDES_FAST_HEAP -DSTM32F40_41xxx -DSYSCLK_FREQ=168000000 -DHSE_VALUE=8000000 -DUSE_STDPERIPH_DRIVER -DARM_MATH_CM4 -D__FPU_PRESENT=1  -DMODULE_SENSORS_BUILTIN  -DMODULE_STATEESTIMATION_BUILTIN  -DMODULE_AIRSPEED_BUILTIN  -DMODULE_STABILIZATION_BUILTIN  -DMODULE_MANUALCONTROL_BUILTIN  -DMODULE_RECEIVER_BUILTIN  -DMODULE_ACTUATOR_BUILTIN  -DMODULE_GPS_BUILTIN  -DMODULE_TXPID_BUILTIN  -DMODULE_CAMERASTAB_BUILTIN  -DMODULE_CAMERACONTROL_BUILTIN  -DMODULE_BATTERY_BUILTIN  -DMODULE_FIRMWAREIAP_BUILTIN  -DMODULE_RADIO_BUILTIN  -DMODULE_PATHPLANNER_BUILTIN  -DMODULE_PATHFOLLOWER_BUILTIN  -DMODULE_OSDOUTOUT_BUILTIN  -DMODULE_LOGGING_BUILTIN  -DMODULE_TELEMETRY_BUILTIN  -DMODULE_NOTIFY_BUILTIN  -DHAS_AUTOTUNE_MODULE  -DHAS_COMUSBBRIDGE_MODULE  -DHAS_UAVOHOTTBRIDGE_MODULE  -DHAS_UAVOMSPBRIDGE_MODULE  -DHAS_UAVOMAVLINKBRIDGE_MODULE  -DHAS_UAVOFRSKYSENSORHUBBRIDGE_MODULE  -DHAS_SENSORS_MODULE  -DHAS_STATEESTIMATION_MODULE  -DHAS_AIRSPEED_MODULE  -DHAS_STABILIZATION_MODULE  -DHAS_MANUALCONTROL_MODULE  -DHAS_RECEIVER_MODULE  -DHAS_ACTUATOR_MODULE  -DHAS_GPS_MODULE  -DHAS_TXPID_MODULE  -DHAS_CAMERASTAB_MODULE  -DHAS_CAMERACONTROL_MODULE  -DHAS_BATTERY_MODULE  -DHAS_FIRMWAREIAP_MODULE  -DHAS_RADIO_MODULE  -DHAS_PATHPLANNER_MODULE  -DHAS_PATHFOLLOWER_MODULE  -DHAS_OSDOUTOUT_MODULE  -DHAS_LOGGING_MODULE  -DHAS_TELEMETRY_MODULE  -DHAS_NOTIFY_MODULE -DUSE_STM32F4xx_RM -DBOARD_TYPE=0x09 -DBOARD_REVISION=0x03 -DHW_TYPE=0x00 -DBOOTLOADER_VERSION=0x06 -DFW_BANK_BASE=0x08020000   -DFW_BANK_SIZE=0x000A0000   -DFW_DESC_SIZE=0x00000064 -DBL_BANK_BASE=0x08000000   -DBL_BANK_SIZE=0x00008000   -DBL_DESC_SIZE= -DEE_BANK_BASE=0x08008000   -DEE_BANK_SIZE=0x00008000

OPT = -Og -gdwarf

CFLAGS  = -mthumb -ffast-math  -mcpu=cortex-m4 -march=armv7e-m -mfpu=fpv4-sp-d16 -mfloat-abi=hard
CFLAGS += $(OPT)
CFLAGS += -mapcs-frame -fomit-frame-pointer
CFLAGS += -Wall -Wextra -Wfloat-equal -Wdouble-promotion -Wshadow -Werror
CFLAGS += -fdata-sections -ffunction-sections
CFLAGS += -std=gnu99 -Wunsuffixed-float-constants -DPIOS_ENABLE_CXX

CPPFLAGS  = -mthumb -ffast-math  -mcpu=cortex-m4 -march=armv7e-m -mfpu=fpv4-sp-d16 -mfloat-abi=hard
CPPFLAGS += $(OPT)
CPPFLAGS += -mapcs-frame -fomit-frame-pointer
CPPFLAGS += -Wall -Wextra -Wfloat-equal -Wdouble-promotion -Wshadow -Werror
CPPFLAGS += -fdata-sections -ffunction-sections
CPPFLAGS += -DPIOS_ENABLE_CXX -fno-rtti -fno-exceptions -std=c++11 -fno-use-cxa-atexit

INCLUDES  = \
-I./flight/libraries/rscode/ \
-I./flight/pios/common/libraries/FreeRTOS//Source/include \
-I./flight/pios/stm32f4xx/inc \
-I./flight/pios/common/libraries/CMSIS/Include \
-I./flight/pios/stm32f4xx/libraries/CMSIS/Device/ST/STM32F4xx/Include \
-I./flight/pios/stm32f4xx/libraries/STM32F4xx_StdPeriph_Driver/inc \
-I./flight/pios/stm32f4xx//libraries/STM32_USB_OTG_Driver/inc \
-I./flight/pios/stm32f4xx//libraries/STM32_USB_Device_Library/Core/inc \
-I./flight/pios/common/libraries/FreeRTOS//Source/portable/GCC/ARM_CM4F \
-I./flight/pios/common/libraries/msheap/ \
-I./flight/pios \
-I./flight/pios/inc \
-I./flight/targets/boards/revolution \
-I./flight/targets/boards/revolution/firmware \
-I./flight/libraries/inc \
-I./flight/pios/common \
-I./flight/targets/boards/revolution/firmware/inc \
-I./flight/libraries/math \
-I./flight/libraries/pid \
-I./flight/uavobjects/inc \
-I./flight/uavtalk/inc \
-I./build/firmware/uavobjects \
-I./flight/libraries/mavlink/v1.0/common \
-I./flight/modules/AutoTune/inc \
-I./flight/modules/ComUsbBridge/inc \
-I./flight/modules/UAVOHottBridge/inc \
-I./flight/modules/UAVOMSPBridge/inc \
-I./flight/modules/UAVOMavlinkBridge/inc \
-I./flight/modules/UAVOFrSKYSensorHubBridge/inc \
-I./flight/modules/Sensors/inc \
-I./flight/modules/StateEstimation/inc \
-I./flight/modules/Airspeed/inc \
-I./flight/modules/Stabilization/inc \
-I./flight/modules/ManualControl/inc \
-I./flight/modules/Receiver/inc \
-I./flight/modules/Actuator/inc \
-I./flight/modules/GPS/inc \
-I./flight/modules/TxPID/inc \
-I./flight/modules/CameraStab/inc \
-I./flight/modules/CameraControl/inc \
-I./flight/modules/Battery/inc \
-I./flight/modules/FirmwareIAP/inc \
-I./flight/modules/Radio/inc \
-I./flight/modules/PathPlanner/inc \
-I./flight/modules/PathFollower/inc \
-I./flight/modules/Osd/osdoutout/inc \
-I./flight/modules/Logging/inc \
-I./flight/modules/Telemetry/inc \
-I./flight/modules/Notify/inc \
-I./flight/modules/System/inc \
-I.

INPUT_FILE = file_list.txt
SRCS := $(file < $(INPUT_FILE))

CSRC := $(filter %.c,$(SRCS))
# $(info ${CSRC})
CPPSRC := $(filter %.cpp,$(SRCS))
# $(info ${CPPSRC})

COBJS  := $(addprefix objs/,$(CSRC:.c=.o))
CPPOBJS := $(addprefix objs/,$(CPPSRC:.cpp=.o))

CDEPS  := $(addprefix deps/,$(CSRC:.c=.d))
CPPDEPS := $(addprefix deps/,$(CPPSRC:.cpp=.d))

.PHONY: all clean elfclean

all: fw_revolution.elf

# -include $(CDEPS)
# -include $(CPPDEPS)

# seven core automatic variables:
# $@ - filename representing the target.
# $% - filename element of an archive member specification.
# $< - filename of the first prerequisite.
# $? - names of all prerequisites that are newer than the target, separated by spaces
# $^ - filenames of all the prerequisites, separated by spaces
# $+ - filenames of prerequisites separated by spaces includes duplicates
# $* - The stem of the target filename (typically filename without suffix)

# return only directory portion of the value by appending “D” to the symbol, $(@D), $(<D), etc.
# return only file portion of the value by appending "F" to the symbol: $(@F), $(<F), etc.

dirs:
	mkdir -p deps objs
	# touch dirs


#	$(CC) $(CFLAGS) $(DEFINES) $(INCLUDES) -c -o objs/$(*F).o $< -MD -MP -MF deps/$(*F).d
objs/%.o : %.c dirs
	mkdir -p $(@D)
	$(CC) $(CFLAGS) $(DEFINES) $(INCLUDES) -c -o objs/$*.o $< -MD -MP -MF deps/$(*F).d


#	$(CXX) $(CPPFLAGS) $(DEFINES) $(INCLUDES) -c -o objs/$(*F).o $< -MD -MP -MF deps/$(*F).d
objs/%.o : %.cpp dirs
	mkdir -p $(@D)
	$(CXX) $(CPPFLAGS) $(DEFINES) $(INCLUDES) -c -o objs/$*.o $< -MD -MP -MF deps/$(*F).d

LDFILES = -T./flight/pios/stm32f4xx/link_STM32F4xx_RM_fw_memory.ld -T./flight/pios/stm32f4xx/link_STM32F4xx_RM_sections.ld -nostartfiles
LDFLAGS1 = -Wl,--warn-common,--fatal-warnings,--sort-common,--sort-section=alignment,--gc-sections
LDFLAGS2 = -lm -lc -lgcc -Wl,-static

fw_revolution.elf: $(CPPOBJS) $(COBJS)
	$(file >$@.input_files) $(foreach O,$^,$(file >>$@.input_files,$O))	
	$(CXX) $(CPPFLAGS) $(DEFINES) $(INCLUDES) @$@.input_files --output $@ $(LDFILES) $(LDFLAGS1) -Wl,-Map=$@.map,--cref $(LDFLAGS2)
	size $@
	$(OBJCOPY) -O ihex $@ $(basename $@).hex

clean:
	rm -f dirs
	rm -f *.elf *.input_files *.map *.hex
	rm -rf objs
	rm -rf deps

elfclean:
	rm -f *.elf *.input_files *.map *.hex