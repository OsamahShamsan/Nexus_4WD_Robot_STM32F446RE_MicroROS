################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/app/utils/math_helpers.c 

OBJS += \
./Core/Src/app/utils/math_helpers.o 

C_DEPS += \
./Core/Src/app/utils/math_helpers.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/app/utils/%.o Core/Src/app/utils/%.su Core/Src/app/utils/%.cyclo: ../Core/Src/app/utils/%.c Core/Src/app/utils/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F446xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -IC:/Users/engsh/Desktop/BA/Nexus_STM32F4_ROS2/NexusRobot_ROS2_Slave/micro_ros_stm32cubemx_utils-kilted/microros_static_library_ide/libmicroros/include -I"C:/Users/engsh/Desktop/BA/Nexus_STM32F4_ROS2/NexusRobot_ROS2_Slave/Core/Src/app/control" -I"C:/Users/engsh/Desktop/BA/Nexus_STM32F4_ROS2/NexusRobot_ROS2_Slave/Core/Src/app/config" -I"C:/Users/engsh/Desktop/BA/Nexus_STM32F4_ROS2/NexusRobot_ROS2_Slave/Core/Src/app/localization" -I"C:/Users/engsh/Desktop/BA/Nexus_STM32F4_ROS2/NexusRobot_ROS2_Slave/Core/Src/app/sensors" -I"C:/Users/engsh/Desktop/BA/Nexus_STM32F4_ROS2/NexusRobot_ROS2_Slave/Core/Src/app/utils" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-app-2f-utils

clean-Core-2f-Src-2f-app-2f-utils:
	-$(RM) ./Core/Src/app/utils/math_helpers.cyclo ./Core/Src/app/utils/math_helpers.d ./Core/Src/app/utils/math_helpers.o ./Core/Src/app/utils/math_helpers.su

.PHONY: clean-Core-2f-Src-2f-app-2f-utils

