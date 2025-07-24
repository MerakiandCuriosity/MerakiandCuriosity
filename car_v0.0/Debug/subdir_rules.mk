################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

# Each subdirectory must supply rules for building sources it contributes
build-443589073: ../Car.syscfg
	@echo 'Building file: "$<"'
	@echo 'Invoking: SysConfig'
	"D:/ti/ccs2011/ccs/utils/sysconfig_1.23.0/sysconfig_cli.bat" --script "D:/ccs_proj/car_v0.0/Car.syscfg" -o "." -s "D:/ti/mspm0_sdk_2_05_00_05/.metadata/product.json" --compiler ticlang
	@echo 'Finished building: "$<"'
	@echo ' '

device_linker.cmd: build-443589073 ../Car.syscfg
device.opt: build-443589073
device.cmd.genlibs: build-443589073
ti_msp_dl_config.c: build-443589073
ti_msp_dl_config.h: build-443589073
Event.dot: build-443589073

%.o: ./%.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: Arm Compiler'
	"D:/ti/ccs2011/ccs/tools/compiler/ti-cgt-armllvm_4.0.2.LTS/bin/tiarmclang.exe" -c @"device.opt"  -march=thumbv6m -mcpu=cortex-m0plus -mfloat-abi=soft -mlittle-endian -mthumb -O2 -I"D:/ccs_proj/car_v0.0" -I"D:/ccs_proj/car_v0.0/Hardware/OLED" -I"D:/ccs_proj/car_v0.0/Hardware/MPU6050" -I"D:/ccs_proj/car_v0.0/Hardware" -I"D:/ccs_proj/car_v0.0/Debug" -I"D:/ti/mspm0_sdk_2_05_00_05/source/third_party/CMSIS/Core/Include" -I"D:/ti/mspm0_sdk_2_05_00_05/source" -gdwarf-3 -MMD -MP -MF"$(basename $(<F)).d_raw" -MT"$(@)"  $(GEN_OPTS__FLAG) -o"$@" "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

startup_mspm0g350x_ticlang.o: D:/ti/mspm0_sdk_2_05_00_05/source/ti/devices/msp/m0p/startup_system_files/ticlang/startup_mspm0g350x_ticlang.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: Arm Compiler'
	"D:/ti/ccs2011/ccs/tools/compiler/ti-cgt-armllvm_4.0.2.LTS/bin/tiarmclang.exe" -c @"device.opt"  -march=thumbv6m -mcpu=cortex-m0plus -mfloat-abi=soft -mlittle-endian -mthumb -O2 -I"D:/ccs_proj/car_v0.0" -I"D:/ccs_proj/car_v0.0/Hardware/OLED" -I"D:/ccs_proj/car_v0.0/Hardware/MPU6050" -I"D:/ccs_proj/car_v0.0/Hardware" -I"D:/ccs_proj/car_v0.0/Debug" -I"D:/ti/mspm0_sdk_2_05_00_05/source/third_party/CMSIS/Core/Include" -I"D:/ti/mspm0_sdk_2_05_00_05/source" -gdwarf-3 -MMD -MP -MF"$(basename $(<F)).d_raw" -MT"$(@)"  $(GEN_OPTS__FLAG) -o"$@" "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

%.o: ../%.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: Arm Compiler'
	"D:/ti/ccs2011/ccs/tools/compiler/ti-cgt-armllvm_4.0.2.LTS/bin/tiarmclang.exe" -c @"device.opt"  -march=thumbv6m -mcpu=cortex-m0plus -mfloat-abi=soft -mlittle-endian -mthumb -O2 -I"D:/ccs_proj/car_v0.0" -I"D:/ccs_proj/car_v0.0/Hardware/OLED" -I"D:/ccs_proj/car_v0.0/Hardware/MPU6050" -I"D:/ccs_proj/car_v0.0/Hardware" -I"D:/ccs_proj/car_v0.0/Debug" -I"D:/ti/mspm0_sdk_2_05_00_05/source/third_party/CMSIS/Core/Include" -I"D:/ti/mspm0_sdk_2_05_00_05/source" -gdwarf-3 -MMD -MP -MF"$(basename $(<F)).d_raw" -MT"$(@)"  $(GEN_OPTS__FLAG) -o"$@" "$<"
	@echo 'Finished building: "$<"'
	@echo ' '


