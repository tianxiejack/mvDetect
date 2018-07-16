################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../include/VIBE/common/ViBeBase.cpp 

OBJS += \
./include/VIBE/common/ViBeBase.o 

CPP_DEPS += \
./include/VIBE/common/ViBeBase.d 


# Each subdirectory must supply rules for building sources it contributes
include/VIBE/common/%.o: ../include/VIBE/common/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: NVCC Compiler'
	/usr/local/cuda-8.0/bin/nvcc -D__MV_DETECT_VIBE_=1 -I../include -I../src/OSA_CAP/inc -I/usr/include/opencv -I../src/MovDetector -I../include/VIBE -G -g -O0 -Xcompiler -fPIC -ccbin aarch64-linux-gnu-g++ -gencode arch=compute_20,code=sm_20 -gencode arch=compute_50,code=sm_50 -m64 -odir "include/VIBE/common" -M -o "$(@:%.o=%.d)" "$<"
	/usr/local/cuda-8.0/bin/nvcc -D__MV_DETECT_VIBE_=1 -I../include -I../src/OSA_CAP/inc -I/usr/include/opencv -I../src/MovDetector -I../include/VIBE -G -g -O0 -Xcompiler -fPIC --compile -m64 -ccbin aarch64-linux-gnu-g++  -x c++ -o  "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


