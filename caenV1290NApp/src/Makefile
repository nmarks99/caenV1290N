TOP=../..

include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

#==================================================
# build a support library

LIBRARY_IOC += caenV1290N

# xxxRecord.h will be created from xxxRecord.dbd
#DBDINC += xxxRecord
# install drvCaenV1290N.dbd into <top>/dbd
DBD += caenV1290NSupport.dbd

caenV1290NSupport_DBD += drvCaenV1290N.dbd

# specify all source files to be compiled and added to the library
caenV1290N_SRCS += drvCaenV1290N.cpp

caenV1290N_LIBS += asyn
caenV1290N_LIBS += $(EPICS_BASE_IOC_LIBS)

# ifeq ($(OS_CLASS), RTEMS)
  # $(PROD_NAME)_SRCS  += $(DBD_NAME)_registerRecordDeviceDriver.cpp
  # $(PROD_NAME)_SRCS  += drvCAEN_V1290N.cpp
# else
  # $(PROD_NAME)_SRCS  += $(DBD_NAME)_registerRecordDeviceDriver.cpp $(PROD_NAME)Main.cpp
# endif
#
# ifeq ($(OS_CLASS), vxWorks)
  # $(DBD_NAME)_DBD += drvCAEN_V1290N.dbd
# endif

#===========================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE
