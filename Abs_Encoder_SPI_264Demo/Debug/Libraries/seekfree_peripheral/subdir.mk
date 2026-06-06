################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
"../Libraries/seekfree_peripheral/SEEKFREE_18TFT.c" \
"../Libraries/seekfree_peripheral/SEEKFREE_7725.c" \
"../Libraries/seekfree_peripheral/SEEKFREE_7725_UART.c" \
"../Libraries/seekfree_peripheral/SEEKFREE_ABSOLUTE_ENCODER.c" \
"../Libraries/seekfree_peripheral/SEEKFREE_BLUETOOTH_CH9141.c" \
"../Libraries/seekfree_peripheral/SEEKFREE_FONT.c" \
"../Libraries/seekfree_peripheral/SEEKFREE_FUN.c" \
"../Libraries/seekfree_peripheral/SEEKFREE_GPS_TAU1201.c" \
"../Libraries/seekfree_peripheral/SEEKFREE_ICM20602.c" \
"../Libraries/seekfree_peripheral/SEEKFREE_IIC.c" \
"../Libraries/seekfree_peripheral/SEEKFREE_IPS114_SPI.c" \
"../Libraries/seekfree_peripheral/SEEKFREE_IPS200_PARALLEL8.c" \
"../Libraries/seekfree_peripheral/SEEKFREE_L3G4200D.c" \
"../Libraries/seekfree_peripheral/SEEKFREE_MMA8451.c" \
"../Libraries/seekfree_peripheral/SEEKFREE_MPU6050.c" \
"../Libraries/seekfree_peripheral/SEEKFREE_MT9V03X.c" \
"../Libraries/seekfree_peripheral/SEEKFREE_OLED.c" \
"../Libraries/seekfree_peripheral/SEEKFREE_RDA5807.c" \
"../Libraries/seekfree_peripheral/SEEKFREE_VIRSCO.c" \
"../Libraries/seekfree_peripheral/SEEKFREE_WIRELESS.c" 

COMPILED_SRCS += \
"Libraries/seekfree_peripheral/SEEKFREE_18TFT.src" \
"Libraries/seekfree_peripheral/SEEKFREE_7725.src" \
"Libraries/seekfree_peripheral/SEEKFREE_7725_UART.src" \
"Libraries/seekfree_peripheral/SEEKFREE_ABSOLUTE_ENCODER.src" \
"Libraries/seekfree_peripheral/SEEKFREE_BLUETOOTH_CH9141.src" \
"Libraries/seekfree_peripheral/SEEKFREE_FONT.src" \
"Libraries/seekfree_peripheral/SEEKFREE_FUN.src" \
"Libraries/seekfree_peripheral/SEEKFREE_GPS_TAU1201.src" \
"Libraries/seekfree_peripheral/SEEKFREE_ICM20602.src" \
"Libraries/seekfree_peripheral/SEEKFREE_IIC.src" \
"Libraries/seekfree_peripheral/SEEKFREE_IPS114_SPI.src" \
"Libraries/seekfree_peripheral/SEEKFREE_IPS200_PARALLEL8.src" \
"Libraries/seekfree_peripheral/SEEKFREE_L3G4200D.src" \
"Libraries/seekfree_peripheral/SEEKFREE_MMA8451.src" \
"Libraries/seekfree_peripheral/SEEKFREE_MPU6050.src" \
"Libraries/seekfree_peripheral/SEEKFREE_MT9V03X.src" \
"Libraries/seekfree_peripheral/SEEKFREE_OLED.src" \
"Libraries/seekfree_peripheral/SEEKFREE_RDA5807.src" \
"Libraries/seekfree_peripheral/SEEKFREE_VIRSCO.src" \
"Libraries/seekfree_peripheral/SEEKFREE_WIRELESS.src" 

C_DEPS += \
"./Libraries/seekfree_peripheral/SEEKFREE_18TFT.d" \
"./Libraries/seekfree_peripheral/SEEKFREE_7725.d" \
"./Libraries/seekfree_peripheral/SEEKFREE_7725_UART.d" \
"./Libraries/seekfree_peripheral/SEEKFREE_ABSOLUTE_ENCODER.d" \
"./Libraries/seekfree_peripheral/SEEKFREE_BLUETOOTH_CH9141.d" \
"./Libraries/seekfree_peripheral/SEEKFREE_FONT.d" \
"./Libraries/seekfree_peripheral/SEEKFREE_FUN.d" \
"./Libraries/seekfree_peripheral/SEEKFREE_GPS_TAU1201.d" \
"./Libraries/seekfree_peripheral/SEEKFREE_ICM20602.d" \
"./Libraries/seekfree_peripheral/SEEKFREE_IIC.d" \
"./Libraries/seekfree_peripheral/SEEKFREE_IPS114_SPI.d" \
"./Libraries/seekfree_peripheral/SEEKFREE_IPS200_PARALLEL8.d" \
"./Libraries/seekfree_peripheral/SEEKFREE_L3G4200D.d" \
"./Libraries/seekfree_peripheral/SEEKFREE_MMA8451.d" \
"./Libraries/seekfree_peripheral/SEEKFREE_MPU6050.d" \
"./Libraries/seekfree_peripheral/SEEKFREE_MT9V03X.d" \
"./Libraries/seekfree_peripheral/SEEKFREE_OLED.d" \
"./Libraries/seekfree_peripheral/SEEKFREE_RDA5807.d" \
"./Libraries/seekfree_peripheral/SEEKFREE_VIRSCO.d" \
"./Libraries/seekfree_peripheral/SEEKFREE_WIRELESS.d" 

OBJS += \
"Libraries/seekfree_peripheral/SEEKFREE_18TFT.o" \
"Libraries/seekfree_peripheral/SEEKFREE_7725.o" \
"Libraries/seekfree_peripheral/SEEKFREE_7725_UART.o" \
"Libraries/seekfree_peripheral/SEEKFREE_ABSOLUTE_ENCODER.o" \
"Libraries/seekfree_peripheral/SEEKFREE_BLUETOOTH_CH9141.o" \
"Libraries/seekfree_peripheral/SEEKFREE_FONT.o" \
"Libraries/seekfree_peripheral/SEEKFREE_FUN.o" \
"Libraries/seekfree_peripheral/SEEKFREE_GPS_TAU1201.o" \
"Libraries/seekfree_peripheral/SEEKFREE_ICM20602.o" \
"Libraries/seekfree_peripheral/SEEKFREE_IIC.o" \
"Libraries/seekfree_peripheral/SEEKFREE_IPS114_SPI.o" \
"Libraries/seekfree_peripheral/SEEKFREE_IPS200_PARALLEL8.o" \
"Libraries/seekfree_peripheral/SEEKFREE_L3G4200D.o" \
"Libraries/seekfree_peripheral/SEEKFREE_MMA8451.o" \
"Libraries/seekfree_peripheral/SEEKFREE_MPU6050.o" \
"Libraries/seekfree_peripheral/SEEKFREE_MT9V03X.o" \
"Libraries/seekfree_peripheral/SEEKFREE_OLED.o" \
"Libraries/seekfree_peripheral/SEEKFREE_RDA5807.o" \
"Libraries/seekfree_peripheral/SEEKFREE_VIRSCO.o" \
"Libraries/seekfree_peripheral/SEEKFREE_WIRELESS.o" 


# Each subdirectory must supply rules for building sources it contributes
"Libraries/seekfree_peripheral/SEEKFREE_18TFT.src":"../Libraries/seekfree_peripheral/SEEKFREE_18TFT.c" "Libraries/seekfree_peripheral/subdir.mk"
	cctc -cs --dep-file="$*.d" --misrac-version=2004 -D__CPU__=tc26xb "-fD:/Github/SmartCar/Abs_Encoder_SPI_264Demo/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc26xb -Y0 -N0 -Z0 -o "$@" "$<"
"Libraries/seekfree_peripheral/SEEKFREE_18TFT.o":"Libraries/seekfree_peripheral/SEEKFREE_18TFT.src" "Libraries/seekfree_peripheral/subdir.mk"
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
"Libraries/seekfree_peripheral/SEEKFREE_7725.src":"../Libraries/seekfree_peripheral/SEEKFREE_7725.c" "Libraries/seekfree_peripheral/subdir.mk"
	cctc -cs --dep-file="$*.d" --misrac-version=2004 -D__CPU__=tc26xb "-fD:/Github/SmartCar/Abs_Encoder_SPI_264Demo/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc26xb -Y0 -N0 -Z0 -o "$@" "$<"
"Libraries/seekfree_peripheral/SEEKFREE_7725.o":"Libraries/seekfree_peripheral/SEEKFREE_7725.src" "Libraries/seekfree_peripheral/subdir.mk"
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
"Libraries/seekfree_peripheral/SEEKFREE_7725_UART.src":"../Libraries/seekfree_peripheral/SEEKFREE_7725_UART.c" "Libraries/seekfree_peripheral/subdir.mk"
	cctc -cs --dep-file="$*.d" --misrac-version=2004 -D__CPU__=tc26xb "-fD:/Github/SmartCar/Abs_Encoder_SPI_264Demo/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc26xb -Y0 -N0 -Z0 -o "$@" "$<"
"Libraries/seekfree_peripheral/SEEKFREE_7725_UART.o":"Libraries/seekfree_peripheral/SEEKFREE_7725_UART.src" "Libraries/seekfree_peripheral/subdir.mk"
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
"Libraries/seekfree_peripheral/SEEKFREE_ABSOLUTE_ENCODER.src":"../Libraries/seekfree_peripheral/SEEKFREE_ABSOLUTE_ENCODER.c" "Libraries/seekfree_peripheral/subdir.mk"
	cctc -cs --dep-file="$*.d" --misrac-version=2004 -D__CPU__=tc26xb "-fD:/Github/SmartCar/Abs_Encoder_SPI_264Demo/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc26xb -Y0 -N0 -Z0 -o "$@" "$<"
"Libraries/seekfree_peripheral/SEEKFREE_ABSOLUTE_ENCODER.o":"Libraries/seekfree_peripheral/SEEKFREE_ABSOLUTE_ENCODER.src" "Libraries/seekfree_peripheral/subdir.mk"
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
"Libraries/seekfree_peripheral/SEEKFREE_BLUETOOTH_CH9141.src":"../Libraries/seekfree_peripheral/SEEKFREE_BLUETOOTH_CH9141.c" "Libraries/seekfree_peripheral/subdir.mk"
	cctc -cs --dep-file="$*.d" --misrac-version=2004 -D__CPU__=tc26xb "-fD:/Github/SmartCar/Abs_Encoder_SPI_264Demo/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc26xb -Y0 -N0 -Z0 -o "$@" "$<"
"Libraries/seekfree_peripheral/SEEKFREE_BLUETOOTH_CH9141.o":"Libraries/seekfree_peripheral/SEEKFREE_BLUETOOTH_CH9141.src" "Libraries/seekfree_peripheral/subdir.mk"
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
"Libraries/seekfree_peripheral/SEEKFREE_FONT.src":"../Libraries/seekfree_peripheral/SEEKFREE_FONT.c" "Libraries/seekfree_peripheral/subdir.mk"
	cctc -cs --dep-file="$*.d" --misrac-version=2004 -D__CPU__=tc26xb "-fD:/Github/SmartCar/Abs_Encoder_SPI_264Demo/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc26xb -Y0 -N0 -Z0 -o "$@" "$<"
"Libraries/seekfree_peripheral/SEEKFREE_FONT.o":"Libraries/seekfree_peripheral/SEEKFREE_FONT.src" "Libraries/seekfree_peripheral/subdir.mk"
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
"Libraries/seekfree_peripheral/SEEKFREE_FUN.src":"../Libraries/seekfree_peripheral/SEEKFREE_FUN.c" "Libraries/seekfree_peripheral/subdir.mk"
	cctc -cs --dep-file="$*.d" --misrac-version=2004 -D__CPU__=tc26xb "-fD:/Github/SmartCar/Abs_Encoder_SPI_264Demo/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc26xb -Y0 -N0 -Z0 -o "$@" "$<"
"Libraries/seekfree_peripheral/SEEKFREE_FUN.o":"Libraries/seekfree_peripheral/SEEKFREE_FUN.src" "Libraries/seekfree_peripheral/subdir.mk"
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
"Libraries/seekfree_peripheral/SEEKFREE_GPS_TAU1201.src":"../Libraries/seekfree_peripheral/SEEKFREE_GPS_TAU1201.c" "Libraries/seekfree_peripheral/subdir.mk"
	cctc -cs --dep-file="$*.d" --misrac-version=2004 -D__CPU__=tc26xb "-fD:/Github/SmartCar/Abs_Encoder_SPI_264Demo/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc26xb -Y0 -N0 -Z0 -o "$@" "$<"
"Libraries/seekfree_peripheral/SEEKFREE_GPS_TAU1201.o":"Libraries/seekfree_peripheral/SEEKFREE_GPS_TAU1201.src" "Libraries/seekfree_peripheral/subdir.mk"
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
"Libraries/seekfree_peripheral/SEEKFREE_ICM20602.src":"../Libraries/seekfree_peripheral/SEEKFREE_ICM20602.c" "Libraries/seekfree_peripheral/subdir.mk"
	cctc -cs --dep-file="$*.d" --misrac-version=2004 -D__CPU__=tc26xb "-fD:/Github/SmartCar/Abs_Encoder_SPI_264Demo/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc26xb -Y0 -N0 -Z0 -o "$@" "$<"
"Libraries/seekfree_peripheral/SEEKFREE_ICM20602.o":"Libraries/seekfree_peripheral/SEEKFREE_ICM20602.src" "Libraries/seekfree_peripheral/subdir.mk"
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
"Libraries/seekfree_peripheral/SEEKFREE_IIC.src":"../Libraries/seekfree_peripheral/SEEKFREE_IIC.c" "Libraries/seekfree_peripheral/subdir.mk"
	cctc -cs --dep-file="$*.d" --misrac-version=2004 -D__CPU__=tc26xb "-fD:/Github/SmartCar/Abs_Encoder_SPI_264Demo/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc26xb -Y0 -N0 -Z0 -o "$@" "$<"
"Libraries/seekfree_peripheral/SEEKFREE_IIC.o":"Libraries/seekfree_peripheral/SEEKFREE_IIC.src" "Libraries/seekfree_peripheral/subdir.mk"
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
"Libraries/seekfree_peripheral/SEEKFREE_IPS114_SPI.src":"../Libraries/seekfree_peripheral/SEEKFREE_IPS114_SPI.c" "Libraries/seekfree_peripheral/subdir.mk"
	cctc -cs --dep-file="$*.d" --misrac-version=2004 -D__CPU__=tc26xb "-fD:/Github/SmartCar/Abs_Encoder_SPI_264Demo/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc26xb -Y0 -N0 -Z0 -o "$@" "$<"
"Libraries/seekfree_peripheral/SEEKFREE_IPS114_SPI.o":"Libraries/seekfree_peripheral/SEEKFREE_IPS114_SPI.src" "Libraries/seekfree_peripheral/subdir.mk"
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
"Libraries/seekfree_peripheral/SEEKFREE_IPS200_PARALLEL8.src":"../Libraries/seekfree_peripheral/SEEKFREE_IPS200_PARALLEL8.c" "Libraries/seekfree_peripheral/subdir.mk"
	cctc -cs --dep-file="$*.d" --misrac-version=2004 -D__CPU__=tc26xb "-fD:/Github/SmartCar/Abs_Encoder_SPI_264Demo/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc26xb -Y0 -N0 -Z0 -o "$@" "$<"
"Libraries/seekfree_peripheral/SEEKFREE_IPS200_PARALLEL8.o":"Libraries/seekfree_peripheral/SEEKFREE_IPS200_PARALLEL8.src" "Libraries/seekfree_peripheral/subdir.mk"
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
"Libraries/seekfree_peripheral/SEEKFREE_L3G4200D.src":"../Libraries/seekfree_peripheral/SEEKFREE_L3G4200D.c" "Libraries/seekfree_peripheral/subdir.mk"
	cctc -cs --dep-file="$*.d" --misrac-version=2004 -D__CPU__=tc26xb "-fD:/Github/SmartCar/Abs_Encoder_SPI_264Demo/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc26xb -Y0 -N0 -Z0 -o "$@" "$<"
"Libraries/seekfree_peripheral/SEEKFREE_L3G4200D.o":"Libraries/seekfree_peripheral/SEEKFREE_L3G4200D.src" "Libraries/seekfree_peripheral/subdir.mk"
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
"Libraries/seekfree_peripheral/SEEKFREE_MMA8451.src":"../Libraries/seekfree_peripheral/SEEKFREE_MMA8451.c" "Libraries/seekfree_peripheral/subdir.mk"
	cctc -cs --dep-file="$*.d" --misrac-version=2004 -D__CPU__=tc26xb "-fD:/Github/SmartCar/Abs_Encoder_SPI_264Demo/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc26xb -Y0 -N0 -Z0 -o "$@" "$<"
"Libraries/seekfree_peripheral/SEEKFREE_MMA8451.o":"Libraries/seekfree_peripheral/SEEKFREE_MMA8451.src" "Libraries/seekfree_peripheral/subdir.mk"
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
"Libraries/seekfree_peripheral/SEEKFREE_MPU6050.src":"../Libraries/seekfree_peripheral/SEEKFREE_MPU6050.c" "Libraries/seekfree_peripheral/subdir.mk"
	cctc -cs --dep-file="$*.d" --misrac-version=2004 -D__CPU__=tc26xb "-fD:/Github/SmartCar/Abs_Encoder_SPI_264Demo/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc26xb -Y0 -N0 -Z0 -o "$@" "$<"
"Libraries/seekfree_peripheral/SEEKFREE_MPU6050.o":"Libraries/seekfree_peripheral/SEEKFREE_MPU6050.src" "Libraries/seekfree_peripheral/subdir.mk"
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
"Libraries/seekfree_peripheral/SEEKFREE_MT9V03X.src":"../Libraries/seekfree_peripheral/SEEKFREE_MT9V03X.c" "Libraries/seekfree_peripheral/subdir.mk"
	cctc -cs --dep-file="$*.d" --misrac-version=2004 -D__CPU__=tc26xb "-fD:/Github/SmartCar/Abs_Encoder_SPI_264Demo/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc26xb -Y0 -N0 -Z0 -o "$@" "$<"
"Libraries/seekfree_peripheral/SEEKFREE_MT9V03X.o":"Libraries/seekfree_peripheral/SEEKFREE_MT9V03X.src" "Libraries/seekfree_peripheral/subdir.mk"
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
"Libraries/seekfree_peripheral/SEEKFREE_OLED.src":"../Libraries/seekfree_peripheral/SEEKFREE_OLED.c" "Libraries/seekfree_peripheral/subdir.mk"
	cctc -cs --dep-file="$*.d" --misrac-version=2004 -D__CPU__=tc26xb "-fD:/Github/SmartCar/Abs_Encoder_SPI_264Demo/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc26xb -Y0 -N0 -Z0 -o "$@" "$<"
"Libraries/seekfree_peripheral/SEEKFREE_OLED.o":"Libraries/seekfree_peripheral/SEEKFREE_OLED.src" "Libraries/seekfree_peripheral/subdir.mk"
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
"Libraries/seekfree_peripheral/SEEKFREE_RDA5807.src":"../Libraries/seekfree_peripheral/SEEKFREE_RDA5807.c" "Libraries/seekfree_peripheral/subdir.mk"
	cctc -cs --dep-file="$*.d" --misrac-version=2004 -D__CPU__=tc26xb "-fD:/Github/SmartCar/Abs_Encoder_SPI_264Demo/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc26xb -Y0 -N0 -Z0 -o "$@" "$<"
"Libraries/seekfree_peripheral/SEEKFREE_RDA5807.o":"Libraries/seekfree_peripheral/SEEKFREE_RDA5807.src" "Libraries/seekfree_peripheral/subdir.mk"
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
"Libraries/seekfree_peripheral/SEEKFREE_VIRSCO.src":"../Libraries/seekfree_peripheral/SEEKFREE_VIRSCO.c" "Libraries/seekfree_peripheral/subdir.mk"
	cctc -cs --dep-file="$*.d" --misrac-version=2004 -D__CPU__=tc26xb "-fD:/Github/SmartCar/Abs_Encoder_SPI_264Demo/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc26xb -Y0 -N0 -Z0 -o "$@" "$<"
"Libraries/seekfree_peripheral/SEEKFREE_VIRSCO.o":"Libraries/seekfree_peripheral/SEEKFREE_VIRSCO.src" "Libraries/seekfree_peripheral/subdir.mk"
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
"Libraries/seekfree_peripheral/SEEKFREE_WIRELESS.src":"../Libraries/seekfree_peripheral/SEEKFREE_WIRELESS.c" "Libraries/seekfree_peripheral/subdir.mk"
	cctc -cs --dep-file="$*.d" --misrac-version=2004 -D__CPU__=tc26xb "-fD:/Github/SmartCar/Abs_Encoder_SPI_264Demo/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc26xb -Y0 -N0 -Z0 -o "$@" "$<"
"Libraries/seekfree_peripheral/SEEKFREE_WIRELESS.o":"Libraries/seekfree_peripheral/SEEKFREE_WIRELESS.src" "Libraries/seekfree_peripheral/subdir.mk"
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"

clean: clean-Libraries-2f-seekfree_peripheral

clean-Libraries-2f-seekfree_peripheral:
	-$(RM) ./Libraries/seekfree_peripheral/SEEKFREE_18TFT.d ./Libraries/seekfree_peripheral/SEEKFREE_18TFT.o ./Libraries/seekfree_peripheral/SEEKFREE_18TFT.src ./Libraries/seekfree_peripheral/SEEKFREE_7725.d ./Libraries/seekfree_peripheral/SEEKFREE_7725.o ./Libraries/seekfree_peripheral/SEEKFREE_7725.src ./Libraries/seekfree_peripheral/SEEKFREE_7725_UART.d ./Libraries/seekfree_peripheral/SEEKFREE_7725_UART.o ./Libraries/seekfree_peripheral/SEEKFREE_7725_UART.src ./Libraries/seekfree_peripheral/SEEKFREE_ABSOLUTE_ENCODER.d ./Libraries/seekfree_peripheral/SEEKFREE_ABSOLUTE_ENCODER.o ./Libraries/seekfree_peripheral/SEEKFREE_ABSOLUTE_ENCODER.src ./Libraries/seekfree_peripheral/SEEKFREE_BLUETOOTH_CH9141.d ./Libraries/seekfree_peripheral/SEEKFREE_BLUETOOTH_CH9141.o ./Libraries/seekfree_peripheral/SEEKFREE_BLUETOOTH_CH9141.src ./Libraries/seekfree_peripheral/SEEKFREE_FONT.d ./Libraries/seekfree_peripheral/SEEKFREE_FONT.o ./Libraries/seekfree_peripheral/SEEKFREE_FONT.src ./Libraries/seekfree_peripheral/SEEKFREE_FUN.d ./Libraries/seekfree_peripheral/SEEKFREE_FUN.o ./Libraries/seekfree_peripheral/SEEKFREE_FUN.src ./Libraries/seekfree_peripheral/SEEKFREE_GPS_TAU1201.d ./Libraries/seekfree_peripheral/SEEKFREE_GPS_TAU1201.o ./Libraries/seekfree_peripheral/SEEKFREE_GPS_TAU1201.src ./Libraries/seekfree_peripheral/SEEKFREE_ICM20602.d ./Libraries/seekfree_peripheral/SEEKFREE_ICM20602.o ./Libraries/seekfree_peripheral/SEEKFREE_ICM20602.src ./Libraries/seekfree_peripheral/SEEKFREE_IIC.d ./Libraries/seekfree_peripheral/SEEKFREE_IIC.o ./Libraries/seekfree_peripheral/SEEKFREE_IIC.src ./Libraries/seekfree_peripheral/SEEKFREE_IPS114_SPI.d ./Libraries/seekfree_peripheral/SEEKFREE_IPS114_SPI.o ./Libraries/seekfree_peripheral/SEEKFREE_IPS114_SPI.src ./Libraries/seekfree_peripheral/SEEKFREE_IPS200_PARALLEL8.d ./Libraries/seekfree_peripheral/SEEKFREE_IPS200_PARALLEL8.o ./Libraries/seekfree_peripheral/SEEKFREE_IPS200_PARALLEL8.src ./Libraries/seekfree_peripheral/SEEKFREE_L3G4200D.d ./Libraries/seekfree_peripheral/SEEKFREE_L3G4200D.o ./Libraries/seekfree_peripheral/SEEKFREE_L3G4200D.src ./Libraries/seekfree_peripheral/SEEKFREE_MMA8451.d ./Libraries/seekfree_peripheral/SEEKFREE_MMA8451.o ./Libraries/seekfree_peripheral/SEEKFREE_MMA8451.src ./Libraries/seekfree_peripheral/SEEKFREE_MPU6050.d ./Libraries/seekfree_peripheral/SEEKFREE_MPU6050.o ./Libraries/seekfree_peripheral/SEEKFREE_MPU6050.src ./Libraries/seekfree_peripheral/SEEKFREE_MT9V03X.d ./Libraries/seekfree_peripheral/SEEKFREE_MT9V03X.o ./Libraries/seekfree_peripheral/SEEKFREE_MT9V03X.src ./Libraries/seekfree_peripheral/SEEKFREE_OLED.d ./Libraries/seekfree_peripheral/SEEKFREE_OLED.o ./Libraries/seekfree_peripheral/SEEKFREE_OLED.src ./Libraries/seekfree_peripheral/SEEKFREE_RDA5807.d ./Libraries/seekfree_peripheral/SEEKFREE_RDA5807.o ./Libraries/seekfree_peripheral/SEEKFREE_RDA5807.src ./Libraries/seekfree_peripheral/SEEKFREE_VIRSCO.d ./Libraries/seekfree_peripheral/SEEKFREE_VIRSCO.o ./Libraries/seekfree_peripheral/SEEKFREE_VIRSCO.src ./Libraries/seekfree_peripheral/SEEKFREE_WIRELESS.d ./Libraries/seekfree_peripheral/SEEKFREE_WIRELESS.o ./Libraries/seekfree_peripheral/SEEKFREE_WIRELESS.src

.PHONY: clean-Libraries-2f-seekfree_peripheral

