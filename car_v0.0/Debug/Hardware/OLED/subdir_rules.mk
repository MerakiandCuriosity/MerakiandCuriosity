################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

# Each subdirectory must supply rules for building sources it contributes
Hardware/OLED/%.o: ../Hardware/OLED/%.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: Arm Compiler'
	"D:/ti/ccs2011/ccs/tools/compiler/ti-cgt-armllvm_4.0.2.LTS/bin/tiarmclang.exe" -c @"device.opt"  -march=thumbv6m -mcpu=cortex-m0plus -mfloat-abi=soft -mlittle-endian -mthumb -O2 -I"D:/ccs_proj/car_v0.0" -I"D:/ccs_proj/car_v0.0/Hardware/OLED" -I"D:/ccs_proj/car_v0.0/Hardware/MPU6050" -I"D:/ccs_proj/car_v0.0/Hardware" -I"D:/ccs_proj/car_v0.0/Debug" -I"D:/ti/mspm0_sdk_2_05_00_05/source/third_party/CMSIS/Core/Include" -I"D:/ti/mspm0_sdk_2_05_00_05/source" -gdwarf-3 -MMD -MP -MF"Hardware/OLED/$(basename $(<F)).d_raw" -MT"$(@)"  $(GEN_OPTS__FLAG) -o"$@" "$<"
	@echo 'Finished building: "$<"'
	@echo ' '


