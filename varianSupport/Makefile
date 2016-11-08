TOP=../
include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

INC += FluoroStructs.h
INC += HcpErrors.h
INC += HcpFuncDefs.h
INC += HcpPrm.h
INC += HcpPxlFormat.h
INC += HcpSundries.h
INC += iostatus.h


# Note, the following files were manually copied from the KsCam distribution on Windows
ifeq (windows-x64, $(findstring windows-x64, $(T_A)))
LIB_INSTALLS    += ../os/windows-x64/VirtCp64.lib

# add 32bit here 
#else ifeq (win32-x86, $(findstring win32-x86, $(T_A)))
#LIB_INSTALLS    += ../os/win32-x86/KsCam.lib

endif

#=============================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE
