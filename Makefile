BUILDDIR		:= build
TARGET			:= mpu6050
SRC				:= main.cpp mpu6050.cpp i2c.cpp
INC				:= ../mylibraries/Common ../mylibraries/Linux
CROSS_COMPILE	:= aarch64-linux-gnu-
LDFLAGS			?= -static

include Makefile.common