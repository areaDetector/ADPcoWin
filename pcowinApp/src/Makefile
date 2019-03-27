TOP=../..

include $(TOP)/configure/CONFIG

# -------------------------------
# Build an Diamond Support Module
# -------------------------------

LIBRARY_IOC += pcowin

# xxxRecord.h will be created from xxxRecord.dbd
#DBDINC += xxx.h

# xxxRecord.dbd will be installed into <top>/dbd
DBD += pcowinSupport.dbd
DBD += pcowinHardwareSupport.dbd

# The following are compiled and added to the support library
pcowin_SRCS += Pco.cpp
pcowin_SRCS += DllApi.cpp
pcowin_SRCS += NDArrayException.cpp
pcowin_SRCS += AsynException.cpp
pcowin_SRCS += StateMachine.cpp
pcowin_SRCS += TraceStream.cpp
pcowin_SRCS += GangServer.cpp
pcowin_SRCS += GangClient.cpp
pcowin_SRCS += GangConnection.cpp
pcowin_SRCS += GangConfig.cpp
pcowin_SRCS += GangMemberConfig.cpp
pcowin_SRCS += GangServerConfig.cpp
pcowin_SRCS += SocketProtocol.cpp
pcowin_SRCS += PerformanceMonitor.cpp
pcowin_SRCS += PcoException.cpp
pcowin_SRCS_WIN32 += PcoApi.cpp
pcowin_SRCS += SimulationApi.cpp
pcowin_SRCS += TakeLock.cpp
pcowin_SRCS += FreeLock.cpp
pcowin_SRCS += IntegerParam.cpp
pcowin_SRCS += DoubleParam.cpp
pcowin_SRCS += StringParam.cpp
pcowin_SRCS += AsynParam.cpp
pcowin_SRCS += ADDriverEx.cpp
pcowin_SRCS += NdArrayRef.cpp

# Include path to vendor headers
USR_INCLUDES_WIN32 += -I../include/

# These are the vendor libraries
ifeq (win32-x86, $(findstring win32-x86, $(T_A)))
LIB_INSTALLS += $(wildcard ../lib/*.lib)
BIN_INSTALLS += $(wildcard ../dll/*.dll)
USR_CFLAGS += -DDEBUG  /Zi /O2
USR_CPPFLAGS += -DDEBUG /Zi /O2
USR_LDFLAGS += /INCREMENTAL:NO /DEBUG /OPT:REF /OPT:ICF
else ifeq (windows-x64-debug, $(findstring windows-x64-debug, $(T_A)))
LIB_INSTALLS += $(wildcard ../lib64/*.lib)
BIN_INSTALLS += $(wildcard ../dll64/*.dll)
USR_CFLAGS += /wd4290
USR_CPPFLAGS += /wd4290
#USR_LDFLAGS += /INCREMENTAL:NO /DEBUG /OPT:REF /OPT:ICF
else ifeq (windows-x64, $(findstring windows-x64, $(T_A)))
LIB_INSTALLS += $(wildcard ../lib64/*.lib)
BIN_INSTALLS += $(wildcard ../dll64/*.dll)
USR_CFLAGS += /O2 /wd4290
USR_CPPFLAGS += /O2 /wd4290
USR_LDFLAGS += /INCREMENTAL:NO /DEBUG /OPT:REF /OPT:ICF
else
USR_CPPFLAGS += -DDEBUG -O0
USR_CFLAGS += -DDEBUG  -O0
endif

#switch off optimization for debugging
#HOST_OPT=NO

# We need to link against the EPICS Base libraries
pcowin_LIBS += $(EPICS_BASE_IOC_LIBS)



include $(TOP)/configure/RULES
