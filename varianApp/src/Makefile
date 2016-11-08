TOP=../..
include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE

PROD_NAME = varianSupport
#======== DETECTOR LIBRARY ==============

LIBRARY_IOC         = varian
LIB_SRCS           += varian.cpp
LIB_LIBS_WIN32     += VirtCp64
LIB_LIBS           += VirtCp64

DBD += varianSupport.dbd

include $(ADCORE)/ADApp/commonLibraryMakefile
#=============================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

