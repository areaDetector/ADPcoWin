TOP=../..

include $(TOP)/configure/CONFIG

# -------------------------------
# Build an Diamond Support Module
# -------------------------------

#LIBRARY_IOC += xxx

# xxxRecord.h will be created from xxxRecord.dbd
#DBDINC += xxx.h

# xxxRecord.dbd will be installed into <top>/dbd
#DBD += xxx.dbd

# The following are compiled and added to the support library
#xxx_SRCS += xxxCodeA.c
#xxx_SRCS += xxxCodeB.c

# We need to link against the EPICS Base libraries
#xxx_LIBS += $(EPICS_BASE_IOC_LIBS)


# ------------------------
# Build an IOC Application
# ------------------------

PROD_IOC = pcocam2

# pcocam2.dbd will be installed into <top>/dbd
DBD += pcocam2.dbd

# pcocam2.dbd will be created from these files
pcocam2_DBD += base.dbd
#pcocam2_DBD += xxx.dbd

# pcocam2_registerRecordDeviceDriver.cpp will be created
# pcocam2.dbd
pcocam2_SRCS += pcocam2_registerRecordDeviceDriver.cpp

# These two lines are needed for non-vxWorks builds, such as Linux
pcocam2_SRCS_DEFAULT += pcocam2Main.cpp
pcocam2_SRCS_vxWorks += -nil-

# Add locally compiled object code
#pcocam2_SRCS +=

# The following adds object code from base/src/vxWorks
pcocam2_OBJS_vxWorks += $(EPICS_BASE_BIN)/vxComLibrary

# This line says that this IOC Application depends on the
# xxx Support Module
#pcocam2_LIBS += xxx

# We need to link this IOC Application against the EPICS Base libraries
pcocam2_LIBS += $(EPICS_BASE_IOC_LIBS)

# ---------------------------------------------------

# NOTE: To build SNL programs, SNCSEQ must be defined
# in the <top>/configure/RELEASE file
#
# The difference between a file with an "st" extension
# and one with an "stt" extension is:
# The "st" extension files get passed through the C preprocessor
# whereas the "stt" files do not.

#ifneq ($(SNCSEQ),)
    # This builds sncExample as a component of pcocam2
#    pcocam2_SNCFLAGS += +r
#    pcocam2_DBD      += sncExample.dbd
#    pcocam2_SRCS     += sncExample.stt
#    pcocam2_LIBS     += seq pv

    # The following builds sncExample as a standalone application
#    PROD_HOST           += sncExample
#    sncExample_SNCFLAGS += +m
#    sncExample_SRCS     += sncExample.stt
#    sncExample_LIBS     += seq pv
    # Here, we need to specify the dependency on EPICS Base libraries
    # again because this is a standalone application and
    # not part of pcocam2
#    sncExample_LIBS     += $(EPICS_BASE_HOST_LIBS)
#endif

include $(TOP)/configure/RULES
