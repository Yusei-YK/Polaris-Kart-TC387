################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
"../code/zxs_imu963RA.c" 

COMPILED_SRCS += \
"code/zxs_imu963RA.src" 

C_DEPS += \
"./code/zxs_imu963RA.d" 

OBJS += \
"code/zxs_imu963RA.o" 


# Each subdirectory must supply rules for building sources it contributes
"code/zxs_imu963RA.src":"../code/zxs_imu963RA.c" "code/subdir.mk"
	cctc -cs --dep-file="$*.d" --misrac-version=2004 -D__CPU__=tc26xb "-fD:/Github/SmartCar/6.6_V1_TC264/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc26xb -Y0 -N0 -Z0 -o "$@" "$<"
"code/zxs_imu963RA.o":"code/zxs_imu963RA.src" "code/subdir.mk"
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"

clean: clean-code

clean-code:
	-$(RM) ./code/zxs_imu963RA.d ./code/zxs_imu963RA.o ./code/zxs_imu963RA.src

.PHONY: clean-code

