################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
"../Libraries/seekfree_libraries/common/SEEKFREE_PRINTF.c" \
"../Libraries/seekfree_libraries/common/common.c" \
"../Libraries/seekfree_libraries/common/zf_assert.c" 

COMPILED_SRCS += \
"Libraries/seekfree_libraries/common/SEEKFREE_PRINTF.src" \
"Libraries/seekfree_libraries/common/common.src" \
"Libraries/seekfree_libraries/common/zf_assert.src" 

C_DEPS += \
"./Libraries/seekfree_libraries/common/SEEKFREE_PRINTF.d" \
"./Libraries/seekfree_libraries/common/common.d" \
"./Libraries/seekfree_libraries/common/zf_assert.d" 

OBJS += \
"Libraries/seekfree_libraries/common/SEEKFREE_PRINTF.o" \
"Libraries/seekfree_libraries/common/common.o" \
"Libraries/seekfree_libraries/common/zf_assert.o" 


# Each subdirectory must supply rules for building sources it contributes
"Libraries/seekfree_libraries/common/SEEKFREE_PRINTF.src":"../Libraries/seekfree_libraries/common/SEEKFREE_PRINTF.c" "Libraries/seekfree_libraries/common/subdir.mk"
	cctc -cs --dep-file="$*.d" --misrac-version=2004 -D__CPU__=tc26xb "-fD:/Github/SmartCar/Abs_Encoder_SPI_264Demo/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc26xb -Y0 -N0 -Z0 -o "$@" "$<"
"Libraries/seekfree_libraries/common/SEEKFREE_PRINTF.o":"Libraries/seekfree_libraries/common/SEEKFREE_PRINTF.src" "Libraries/seekfree_libraries/common/subdir.mk"
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
"Libraries/seekfree_libraries/common/common.src":"../Libraries/seekfree_libraries/common/common.c" "Libraries/seekfree_libraries/common/subdir.mk"
	cctc -cs --dep-file="$*.d" --misrac-version=2004 -D__CPU__=tc26xb "-fD:/Github/SmartCar/Abs_Encoder_SPI_264Demo/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc26xb -Y0 -N0 -Z0 -o "$@" "$<"
"Libraries/seekfree_libraries/common/common.o":"Libraries/seekfree_libraries/common/common.src" "Libraries/seekfree_libraries/common/subdir.mk"
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
"Libraries/seekfree_libraries/common/zf_assert.src":"../Libraries/seekfree_libraries/common/zf_assert.c" "Libraries/seekfree_libraries/common/subdir.mk"
	cctc -cs --dep-file="$*.d" --misrac-version=2004 -D__CPU__=tc26xb "-fD:/Github/SmartCar/Abs_Encoder_SPI_264Demo/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc26xb -Y0 -N0 -Z0 -o "$@" "$<"
"Libraries/seekfree_libraries/common/zf_assert.o":"Libraries/seekfree_libraries/common/zf_assert.src" "Libraries/seekfree_libraries/common/subdir.mk"
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"

clean: clean-Libraries-2f-seekfree_libraries-2f-common

clean-Libraries-2f-seekfree_libraries-2f-common:
	-$(RM) ./Libraries/seekfree_libraries/common/SEEKFREE_PRINTF.d ./Libraries/seekfree_libraries/common/SEEKFREE_PRINTF.o ./Libraries/seekfree_libraries/common/SEEKFREE_PRINTF.src ./Libraries/seekfree_libraries/common/common.d ./Libraries/seekfree_libraries/common/common.o ./Libraries/seekfree_libraries/common/common.src ./Libraries/seekfree_libraries/common/zf_assert.d ./Libraries/seekfree_libraries/common/zf_assert.o ./Libraries/seekfree_libraries/common/zf_assert.src

.PHONY: clean-Libraries-2f-seekfree_libraries-2f-common

