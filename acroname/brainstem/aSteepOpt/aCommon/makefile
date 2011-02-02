# makefile: host/aTEAvm/aCommon

ROOT = ../../

INCLUDES = \
	-I. \
	-I../aUnix \
	-I../../aCommon \
	-I ../../aIO/aCommon \
	-I ../../aIO/aUnix \
	-I ../../aStem/aCommon \
	-I ../../aStem/aUnix \
	-I ../../aSteep/aCommon \
	-I ../../aTEAvm/aCommon

OBJECTDIR = ../unix_aTEAvm_Data

SOURCE = ../../aCommon/aAssert.c \
         ../../aCommon/aMemPool.c \
	 ../../aCommon/aMemHandleDebug.c \
	 ../../aCommon/aMemLeakDebug.c \
	 aSteepOpt.c \
	 aSteepOptInternal.c

CFLAGS = -Wall -DaUNIX -c -fPIC

# include the generic file for all object building
include $(ROOT)object_makefile
