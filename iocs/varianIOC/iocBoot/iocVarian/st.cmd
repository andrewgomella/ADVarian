############################################################################
< envPaths

# For deviocstats
epicsEnvSet("ENGINEER", "Andrew Gomella")
epicsEnvSet("LOCATION", "B1D521D DT-ASUSWIN102")
epicsEnvSet("STARTUP","$(TOP)/iocBoot/$(IOC)")
epicsEnvSet("ST_CMD","st.cmd")

# Change this PV according to set-up
epicsEnvSet "PREFIX" "$(PREFIX=VPFI:VARIAN)"
epicsEnvSet("P", "$(PREFIX)")
epicsEnvSet "CONFIG" "$(CONFIG=detector)"
epicsEnvSet "EPICS_IOC_LOG_INET" "192.168.1.102"
epicsEnvSet "EPICS_IOC_LOG_PORT" "7004"
epicsEnvSet("PORT",   "PS1")
epicsEnvSet("QSIZE",  "20")
epicsEnvSet("XSIZE",  "2816")
epicsEnvSet("YSIZE",  "3584")
epicsEnvSet("NCHANS", "2048")
epicsEnvSet("EPICS_DB_INCLUDE_PATH", "$(ADCORE)/db")
############################################################################
# Increase size of buffer for error logging from default 1256
errlogInit(20000)
############################################################################
# Register all support components
dbLoadDatabase("$(ADVARIAN)/iocs/varianIOC/dbd/varianApp.dbd")
varianApp_registerRecordDeviceDriver pdbbase 
############################################################################
# Connect to varian panel
# Asyn port, Rad config folder, Fluoro config folder, custom1, custom2, Which folder to boot into (0 or 1), 
# whether to run checkLink at every connection (or reconnection), maxBuffers, maxMemory, stackSize
varianConfig("$(PORT)", "246S03-1506-normalMammo\", "246S03-1506-normalFluoro\", "246S03-1506-normalMammo\", "246S03-1506-normalFluoro\", 0, 0, 50, 0, 0 , 0 ,10)
#asynSetTraceIOMask("$(PORT)",0,2)
#asynSetTraceMask("$(PORT)",0,255)
#asynSetTraceMask("$(PORT)",0,255)
############################################################################
# Load record instances
dbLoadRecords("$(ADCORE)/db/ADBase.template","P=$(PREFIX):,R=cam1:,PORT=$(PORT),ADDR=0,TIMEOUT=1")
dbLoadRecords("$(ADCORE)/Db/NDFile.template","P=$(PREFIX):,R=cam1:,PORT=$(PORT),ADDR=0,TIMEOUT=1")

# Note that prosilica.template must be loaded after NDFile.template to replace the file format correctly
dbLoadRecords("$(ADVARIAN)/db/varian.template","P=$(PREFIX):,R=cam1:,PORT=$(PORT),ADDR=0,TIMEOUT=1")

# Create a standard arrays plugin
NDStdArraysConfigure("Image1", 5, 0, "$(PORT)", 0, 0)
dbLoadRecords("$(ADCORE)/db/NDPluginBase.template","P=$(PREFIX):,R=image1:,PORT=Image1,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),NDARRAY_ADDR=0")
dbLoadRecords("$(ADCORE)/db/NDStdArrays.template","P=$(PREFIX):,R=image1:,PORT=Image1,ADDR=0,TIMEOUT=1,TYPE=Int16,FTVL=SHORT,NELEMENTS=10092544")

# Create another standard arrays plugin
NDStdArraysConfigure("Image2", 200, 0, "$(PORT)", 0, 0)
dbLoadRecords("$(ADCORE)/db/NDPluginBase.template","P=$(PREFIX):,R=image2:,PORT=Image2,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),NDARRAY_ADDR=0")
dbLoadRecords("$(ADCORE)/db/NDStdArrays.template", "P=$(PREFIX):,R=image2:,PORT=Image2,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),TYPE=Int16,FTVL=SHORT,NELEMENTS=10092544")

# Load iocAdmin records
dbLoadRecords("$(DEVIOCSTATS)/db/iocAdminSoft.db", "IOC=$(PREFIX)")
############################################################################
cd $(IPL_SUPPORT)
# Load all other plugins for area-detector
< ad_plugins.cmd
# Load save_restore.cmd
< save_restore.cmd
set_requestfile_path("$(ADVARIAN)/varianApp/Db")
asSetFilename("$(IPL_SUPPORT)/security.acf")
############################################################################
# Start EPICS IOC
cd $(STARTUP)
iocInit()
############################################################################
# Start sequence program
seq &ADInitSettings("A=$(P)")
############################################################################
# save things every 5 seconds
create_monitor_set("auto_settings.req", 5,"P=$(PREFIX):")

# Handle autosave 'commands' contained in loaded databases
# Searches through the EPICS database for info nodes named 'autosaveFields' 
# and 'autosaveFields_pass0' and write the PV names to the files 
# 'info_settings.req' and 'info_positions.req'
makeAutosaveFiles()
create_monitor_set("info_positions.req",5,"P=$(PREFIX):")
create_monitor_set("info_settings.req",30,"P=$(PREFIX):")
############################################################################
# Start EPICS IOC log server
iocLogInit()
setIocLogDisable(0)
############################################################################
# Turn on caPutLogging:
# Log values only on change to the iocLogServer:
caPutLogInit("$(EPICS_IOC_LOG_INET):$(EPICS_IOC_LOG_PORT)",1)
caPutLogShow(2)
############################################################################
# Incase auto save has issues set these records on ioc Initialization
# EPICS Area detector Shutter set-up
dbpf("$(PREFIX):cam1:ShutterOpenEPICS.OUT", "VPFI:VarianSync:PaxscanShutter NPP NMS")
dbpf("$(PREFIX):cam1:ShutterCloseEPICS.OUT", "VPFI:VarianSync:PaxscanShutter NPP NMS")
dbpf("$(PREFIX):cam1:ShutterStatusEPICS_RBV.INP", "VPFI:VarianSync:PaxscanShutter NPP NMS")

dbpf("$(PREFIX):cam1:ShutterMode", "1")

dbpf("$(PREFIX):ROI1:MinY", "20")
dbpf("$(PREFIX):cam1:ImageMode", "1")

# Set varian mode exposure time 
dbpf("$(PREFIX):cam1:VarianMode", "1")

# Enable save corrected image and set image mode to single
dbpf("$(PREFIX):cam1:SaveCorrected", "1")
dbpf("$(PREFIX):cam1:ImageMode", "0")
############################################################################
# write all the PV names to a local file
dbl > records.txt
############################################################################
# print the time our boot was finished
date
############################################################################
#system ("python varianhelper.py")
############################################################################