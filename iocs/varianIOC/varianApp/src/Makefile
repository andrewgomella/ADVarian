TOP=../..
include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================
PROD_NAME = varianApp
PROD_IOC += $(PROD_NAME)

# <name>.dbd will be created from <name>Include.dbd
DBD += $(PROD_NAME).dbd
$(PROD_NAME)_DBD += varianSupport.dbd
$(PROD_NAME)_DBD += system.dbd
$(PROD_NAME)_DBD += ADInitSupport.dbd

# <name>_registerRecordDeviceDriver.cpp will be created from <name>.dbd
PROD_SRCS += $(PROD_NAME)_registerRecordDeviceDriver.cpp $(PROD_NAME)Main.cpp

# Add locally compiled object code
PROD_LIBS          += varian
PROD_LIBS_WIN32    += VirtCp64
#PROD_SYS_LIBS      += $(AREA_DETECTOR)\ADVarian-develop\varianApp\src\VirtCp64

include $(ADCORE)/ADApp/commonDriverMakefile

ifdef DEVIOCSTATS
  $(PROD_NAME)_DBD += devIocStats.dbd
  PROD_LIBS += devIocStats
endif
ifdef ALIVE
  $(PROD_NAME)_DBD += aliveSupport.dbd
  PROD_LIBS += alive
endif
ifdef CAPUTLOG
  $(PROD_NAME)_DBD += caPutLog.dbd
  PROD_LIBS += caPutLog
endif
ifdef SNCSEQ
	# seq-2-1 and earlier has devSequencer
	ifeq (seq-2-1, $(findstring seq-2-1, $(SNCSEQ)))
		$(PROD_IOC)_DBD += devSequencer.dbd
		$(PROD_IOC)_LIBS += seqDev seq pv
	else ifeq (seq-2.1, $(findstring seq-2.1, $(SNCSEQ)))
		$(PROD_IOC)_DBD += devSequencer.dbd
		$(PROD_IOC)_LIBS += seqDev seq pv
	# seq-2-2
	else 
		$(PROD_IOC)_LIBS += seq pv
	endif
	$(PROD_NAME)_SRCS += ADInitSettings.st
endif

#=============================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

