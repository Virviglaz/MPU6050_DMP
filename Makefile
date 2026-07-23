BUILDDIR		:= build
TARGET			:= mpu6050
SRC				:= main.cpp
SRC				+= mpu6050.cpp
SRC				+= ../mylibraries/Linux/i2c.cpp
SRC				+= ../mylibraries/Common/imu.cpp
INC				:= ../mylibraries/Common
INC				+= ../mylibraries/Linux
CROSS_COMPILE	:= aarch64-linux-gnu-
LDFLAGS			?= -static

include BuildServices/Makefile.common