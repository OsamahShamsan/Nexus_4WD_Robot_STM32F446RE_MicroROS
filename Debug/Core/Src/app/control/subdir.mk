################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/app/control/motor_driver.c 

OBJS += \
./Core/Src/app/control/motor_driver.o 

C_DEPS += \
./Core/Src/app/control/motor_driver.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/app/control/%.o Core/Src/app/control/%.su Core/Src/app/control/%.cyclo: ../Core/Src/app/control/%.c Core/Src/app/control/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F446xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -IC:/Users/engsh/Desktop/BA/Nexus_STM32F4_ROS2/NexusRobot_ROS2_Slave/micro_ros_stm32cubemx_utils-kilted/microros_static_library_ide/libmicroros/include -I"C:/Users/engsh/Desktop/BA/Nexus_STM32F4_ROS2/NexusRobot_ROS2_Slave/Core/Src/app/control" -I"C:/Users/engsh/Desktop/BA/Nexus_STM32F4_ROS2/NexusRobot_ROS2_Slave/Core/Src/app/config" -I"C:/Users/engsh/Desktop/BA/Nexus_STM32F4_ROS2/NexusRobot_ROS2_Slave/Core/Src/app/localization" -I"C:/Users/engsh/Desktop/BA/Nexus_STM32F4_ROS2/NexusRobot_ROS2_Slave/Core/Src/app/sensors" -I"C:/Users/engsh/Desktop/BA/Nexus_STM32F4_ROS2/NexusRobot_ROS2_Slave/Core/Src/app/utils" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-app-2f-control

clean-Core-2f-Src-2f-app-2f-control:
	-$(RM) ./Core/Src/app/control/motor_driver.cyclo ./Core/Src/app/control/motor_driver.d ./Core/Src/app/control/motor_driver.o ./Core/Src/app/control/motor_driver.su

.PHONY: clean-Core-2f-Src-2f-app-2f-control

