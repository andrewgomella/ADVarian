/* varian.cpp
 *
 * This is a driver for Varian Paxscan 3024M/3024I
 *
 * Author: Andrew Gomella
 *         NIH
 *
 * Written: June 2014
 * Major Updates/Refactoring: June 2015
 *
 * Adapted from Mark River's Prosilica Driver
 * and Varian supplied sample code
 *
 */

//AreaDetector Header file
#include "ADDriver.h"

//Standard includes
#include <stddef.h>
#include <stdlib.h>
#include <typeinfo>

#include <stdio.h>
#include <conio.h>
#include <iostream>

//Epics Includes
#include <epicsTime.h>
#include <epicsThread.h>
#include <epicsString.h>
#include <epicsStdio.h>
#include <epicsMutex.h>
#include <cantProceed.h>
#include <osiSock.h>
#include <iocsh.h>
#include <epicsExport.h>
#include <epicsExit.h>

//Header files from VirtualCP library
#include "HcpErrors.h"
#include "FluoroStructs.h"
#include "HcpFuncDefs.h"
#include "HcpPrm.h"
#include "HcpPxlFormat.h"
#include "HcpSundries.h"
#include "iostatus.h"

/* Varian driver specific parameters */
// Info about current mode
#define VarianConfigString          "VARIAN_CONFIG"
#define VarianModeString            "VARIAN_MODE"
#define VarianAcquisitionTypeString "VARIAN_ACQUISITION_TYPE"
#define NumberOfModesString         "NUMBER_OF_MODES"
#define VarianModeFluoroString      "VARIAN_MODE_FLUORO"
#define VarianModeDescriptionString "VARIAN_MODE_DESCRIPTION"
#define DoubleModeString            "DOUBLE_MODE"
#define DebugModeString             "DEBUG_MODE"

// Link check routine
#define CheckPaxscanLinkString    "CHECK_PAXSCAN_LINK"

// File saving settings for Varian-style radiography
#define SaveRawString             "SAVE_RAW"
#define SaveOffsetString          "SAVE_OFFSET"
#define SaveCorrectedString       "SAVE_CORRECTED"
#define SavePreviewString         "SAVE_PREVIEW"

// Timing info:
#define PrepareTimeString         "PREPARE_TIME"
#define FirstFrameTimeString      "FIRST_FRAME_TIME"
#define SecondFrameTimeString     "SECOND_FRAME_TIME"
#define EndRadiographyTimeString  "END_RADIOGRAPHY_TIME"
#define SingleAcquisitionSequenceTimeString "SINGLE_ACQUISITION_SEQUENCE_TIME"

// Info about Receptor from SqueryProgInfoRcpt
#define PanelTypeString           "PANEL_TYPE"
#define FirmwareVersionString     "FIRMWARE_VERSION"
#define BoardSerialNumberString   "BOARD_SERIAL_NUMBER"

// Info about Receptor from SDeviceInfo1
#define ReceptorConfigPathString  "RECEPTOR_CONFIG_PATH"
#define MacAddressString          "MAC_ADDRESS"
#define IpAddressString           "IP_ADDRESS"

// Info about last acquired frame from SQueryProgInfoFrame
#define ExposedString             "EXPOSED"
#define AcqTimeString             "ACQTIME"
#define FrameTypeString           "FRAMETYPE"
#define ReadyForExposureString    "READYFOREXP"

// Status information
#define VarianStateString         "VARIAN_STATE"
#define VarianPhaseString         "VARIAN_PHASE"
#define HandswitchStateString     "HANDSWITCH_STATE"
#define Temp1String               "TEMP1"
#define Temp2String               "TEMP2"
#define Temp3String               "TEMP3"
#define Temp4String               "TEMP4"
#define Temp5String               "TEMP5"
#define Temp6String               "TEMP6"
#define NumFramesString           "NUM_FRAMES"
#define CompleteString            "COMPLETE"
#define NumPulsesString           "NUM_PULSES"
#define ReadyForPulseString       "READY_FOR_PULSE"

// Mode specific information including stored calibration
#define VarianFrameRateString      "VARIAN_FRAME_RATE"
#define OffsetMedianString         "OFFSET_MEDIAN"
#define OffsetStdDevString         "OFFSET_STD_DEV"
#define GainMedianString           "GAIN_MEDIAN"
#define GainStdDevString           "GAIN_STD_DEV"
#define GainScalingString          "GAIN_SCALING"
#define TimeLastCalibratedString   "TIME_LAST_CALIBRATED"

// Calibration Routines and settings
#define GainCalString              "GAIN_CAL"
#define GainCalNumOffsetsString    "GAIN_CAL_NUM_OFFSETS"
#define GainCalNumFlatsString      "GAIN_CAL_NUM_FLATS"
#define AnalogOffsetCalString      "ANALOG_OFFSET_CAL"
#define OffsetCalibrationString    "OFFSET_CAL"
#define OffsetCalibrationNumString "OFFSET_CAL_NUM"

// Correction settings
#define SetCorrectionsString       "SET_CORRECTIONS"
#define OffsetCorrectionString     "OFFSET_CORRECTION"
#define GainCorrectionString       "GAIN_CORRECTION"
#define DefectCorrectionString     "DEFECT_CORRECTION"
#define LineCorrectionString       "LINE_CORRECTION"
#define FluoroCorrectionsString    "FLUORO_CORRECTIONS"

// Calibration frame retreival
#define ExtractOffsetFrameString    "EXTRACT_OFFSET"
#define ExtractGainFrameString      "EXTRACT_GAIN"
#define ExtractDefectFrameString    "EXTRACT_DEFECT"
#define ExtractAuxDefectFrameString "EXTRACT_AUX_DEFECT"

#define TIMEOUT_UNLIMITED    (0)
#define TIMEOUT_5_SEC        ( 5000)
#define TIMEOUT_10_SEC       (10000)
#define TIMEOUT_12_SEC       (12000)
#define TIMEOUT_15_SEC       (15000)
#define TIMEOUT_30_SEC       (30000)
#define TIMEOUT_60_SEC       (60000)
#define QUERY_WAIT_INTERVAL  (10) //how often to poll detectors status during acquisitions(ms)
#define MAX_HCP_ERROR_CNT    65536

//Varian "states" used to update user about current status
#define VARIAN_READY 0
#define VARIAN_RADIOGRAPHY 1
#define VARIAN_FLUOROSCOPY 2
#define VARIAN_RADIOGRAPHY_CAL 3
#define VARIAN_FLUOROSCOPY_CAL 4
#define VARIAN_CHECKLINK 5
#define VARIAN_CONNECTING 6
#define VARIAN_DISCONNECTED 7
#define VARIAN_CHANGEMODE 8

using namespace std;
static const char *driverName = "varian";

//////////////////////////////////////////////////////////////

/** Driver for Varian 3024m using the VirtualCP library */
class varian : public ADDriver {
public:
    /* Constructor and Destructor */
    varian(const char *portName, const char *gszIniPathRad,
           const char *gszIniPathFluoro, const char *gszIniPathCustom1,
           const char *gszIniPathCustom2, int bootIniPath,
           int checkLinkOnConnect, int maxBuffers, size_t maxMemory,
           int priority, int stackSize);
    ~varian();

    /* These methods are overwritten from asynPortDriver */
    virtual asynStatus connect(asynUser* pasynUser);
    virtual asynStatus disconnect(asynUser* pasynUser);

    /* These are the methods that we override from ADDriver */
    virtual asynStatus writeInt32(asynUser *pasynUser, epicsInt32 value);

    /* Disconnects the camera */
    static void shutdown(void *arg);

    /* Function called from VirtualCP if connection is lost */
    void connectionLostCallback();

    /* Image Grab Threads */
    void imageGrabTask();
    void fluoroGrabTask();

protected:
    int VarianConfig;
#define FIRST_VARIAN_PARAM VarianConfig
    int VarianMode;
    int VarianAcquisitionType;
    int NumberOfModes;
    int VarianModeFluoro;
    int VarianModeDescription;
    int DoubleMode;
    int DebugMode;
    int SetCorrections;
    int OffsetCorrection;
    int GainCorrection;
    int DefectCorrection;
    int FluoroCorrections;
    int PrepareTime;
    int FirstFrameTime;
    int SecondFrameTime;
    int EndRadiographyTime;
    int SaveRaw;
    int SaveOffset;
    int SaveCorrected;
    int SavePreview;
    int FirmwareVersion;
    int BoardSerialNumber;
    int ReceptorConfigPath;
    int MacAddress;
    int IpAddress;
    int Complete;
    int ReadyForPulse;
    int NumFrames;
    int NumPulses;
    int VarianState;
    int VarianPhase;
    int HandswitchState;
    int Temp1;
    int Temp2;
    int Temp3;
    int Temp4;
    int Temp5;
    int Temp6;
    int ImagingMode;
    int CheckPaxscanLink;
    int OffsetMedian;
    int OffsetStdDev;
    int GainMedian;
    int GainStdDev;
    int GainScaling;
    int GainCal;
    int TimeLastCalibrated;
    int AnalogOffsetCal;
    int GainCalNumOffsets;
    int GainCalNumFlats;
    int ExtractOffsetFrame;
    int ExtractGainFrame;
    int ExtractDefectFrame;
    int ExtractAuxDefectFrame;
    int VarianFrameRate;
#define LAST_VARIAN_PARAM VarianFrameRate
#define NUM_VARIAN_PARAMS ((int)(&LAST_VARIAN_PARAM - &FIRST_VARIAN_PARAM + 1))

private:
    /* These are the methods that are new to this class */
    asynStatus readParameters();
    asynStatus connectCamera();
    asynStatus disconnectCamera();
    asynStatus startCapture();
    asynStatus stopCapture();

    /* Varian timing methods */
    DWORD gdwStartTickCount;
    char  gszTickCounts[10][256];
    int   gnTickCountIdx;
    DWORD gdwTick[10];
    void  PrintTickCounts();
    void  StartTickCount();
    void  EndTickCount(char* szTickCountMsg);
    void  ResetTickCounts();
    int QueryProgress(UQueryProgInfo &CurrentStatus);
    int QueryWaitOnStandby(UQueryProgInfo &CurrentStatus,
                           int TimeoutMSec,
                           bool *CancelFlagPtr);
    int QueryWaitOnNumFrames(UQueryProgInfo &CurrentStatus,
                             int TargetNumFrames,
                             int framesPerExposed,
                             int TimeoutMSec,
                             bool *CancelFlagPtr);
    int QueryWaitOnNumPulses(UQueryProgInfo &CurrentStatus,
                             int TargetNumberPulses,
                             int TimeoutMSec,
                             bool *CancelFlagPtr);
    int QueryWaitOnReadyForPulse(UQueryProgInfo &CurrentStatus,
                                 int TargetState,
                                 int TimeoutMSec,
                                 bool* CancelFlagPtr,
                                 bool bVerbose);
    int QueryWaitOnComplete(UQueryProgInfo &CurrentStatus,
                            int TimeoutMSec,
                            bool *CancelFlagPtr);

    /* from varian fluoro example */
    asynStatus performFluoroAcquisition();

    /* routines */
    bool DoCancel();
    int doHandSwitch(int newHSState);
    void changeConfig(int configNum);
    void changeDebug(int newDebugMode);
    void changeMode(int newModeNumber);
    void changeCorrections();
    int toggleMode();

    void checkLink();
    void performGainCal();
    void performGainCalRad();
    void performGainCalPrePulse();
    void performGainCalFluoro();
    int  performRadAnalogOffsetCal();


    /* radiography functions */
    void  performSwRadAcquisition();
    asynStatus performRadAcquisition();
    asynStatus performDoubleRadAcquisition();
    int radFirstFrame(UQueryProgInfo CurrentStatus);
    void radSecondFrame(UQueryProgInfo CurrentStatus);
    void endRadiography(UQueryProgInfo CurrentStatus);
    void continueAcquisition(int varFramesPerAcq);

    /* utilities */
    void updateVarianState(int newVarianState);
    void printAsynMessage(char* message, char* functionName,
                          int vcpResult);
    void GetCPError(int ErrorCode, char* ErrorMsg);
    void getFrameName(int frameCode, char* frameName);
    void extractFrame(int varianImageType);
    void sendFrameToEpics(USHORT *pFrame, int varianImageType);

    SModeInfo  gModeInfo; //struct containing current mode info
    char   *gszIniPath; //current config folder location
    char   *gszIniPathVarianRad; //folder where we store radiography config files
    char   *gszIniPathVarianFluoro; //folder where we store fluoroscopy config files
    char   *gszIniPathCustom1; // custom config folder 1
    char   *gszIniPathCustom2; // custom config folder 2
    bool   checkLinkOnConnect; //whether to checkLink everytime we reconnect
    DWORD  gdwTickSelectMode;
    char   gszTickCountsSelectMode[256];
    bool   modeHasTwoFrames; //true if mode has two frames
    int    modeNumber;  //current VarianModeNumber
    int    modeNFramesPerExposed;
    int    modeNFramesPerOffset;
    int    numModes;    //number of available modes
    int    maxPixelValue;
    time_t lastCalTime; //time last calibrated
    BOOL   acqInProg; // true when acq in progress
    int    varianState;
    int    varianPhase;

    int sensorWidth;    // width of sensor, fixed
    int sensorHeight;   // height of sensor, fixed
    int sensorSize;     //area of the panel- sensorWidth*sensorHeight
    int radMode;        //if 1, current mode is radiography, if 0 it is fluoro
    int varianAcqType;  // mammomodetypeflag, 0 if radiography, 2/3 if prepulse, 4 if fluoro
    int currConfigNum;
    unsigned int maxFrameSize; //sensor size * bit depth(16)
    NDArray *pImage; // AreaDetector pointer to image data
    size_t dims[2]; // size of NDArray
    epicsEventId startEventId_; //signals acquisition start

    bool masterCancelFlag; //master cancel flag

    /* fluoro mode variables */
    char        gFrameReadyName[MAX_STR];
    bool        GGrabbingIsActive = false;
    SLivePrms*  GLiveParams = NULL;
    long        GNumFrames = 0;
    long        GImgDisplayThreadIsAlive = 0;

};

varian::~varian() {
    this->lock();
    disconnectCamera();
    this->unlock();
}

void varian::shutdown (void* arg) {
    varian *v = (varian*)arg;
    if (v) delete v;
}

static void imageGrabTaskC(void *drvPvt)
{
    varian *pPvt = (varian *)drvPvt;
    pPvt->imageGrabTask();
}

static void fluoroGrabTaskC(void *drvPvt)
{
    varian *pPvt = (varian *)drvPvt;
    pPvt->fluoroGrabTask();
}

static void connectionLostCallbackC(void *drvPvt)
{
    varian *pPvt = (varian *)drvPvt;
    pPvt->connectionLostCallback();
}

/*beyond ADStatus, need to keep track of what "state" the panel is in
 mainly we are concerned if an acquisition, or calibration is in progress*/
void varian::updateVarianState(int newVarianState)
{
    this->varianState = newVarianState;
    setIntegerParam(VarianState, newVarianState);
    callParamCallbacks();
    return;
}

/* Verify integrity of Paxscan link by using the built in
   vip_check_link function which obtains a dark frame
   and analyzes it against reference numbers           */
void varian::checkLink()
{
    static char *functionName = "checkLink";
    //update state to "checklink"
    updateVarianState(5);

    // Clear the structure and set the structure size member
    SCheckLink CheckLinkData;
    memset(&CheckLinkData, 0, sizeof(SCheckLink));
    CheckLinkData.StructSize = sizeof(SCheckLink);
    // Two possibilities HCP_CHKLNK_LONG and HCP_CHKLNK_SHORT
    CheckLinkData.ChkLnkType = HCP_CHKLNK_SHRT;

    printf("\nRunning vip_check_link to verify proper link to Paxscan\n");
    int result = vip_check_link(&CheckLinkData);
    printAsynMessage("vip_check_link()", functionName, result);
    if (result != HCP_NO_ERR){
        setIntegerParam(ADStatus, ADStatusError);
        return;
    }

    if (result == HCP_NO_ERR)
        printAsynMessage("Check Link was successful.", functionName, result);

    if (result == HCP_NO_ERR && this->modeNumber >= 0 &&
            CheckLinkData.ChkBufPtr)
    {
        //Here it is possible to send the frame used in the vip_check_link routine to EPICS
        // this will be a "dark field" (offset) image.
        //sendFrameToEpics((USHORT*)CheckLinkData.ChkBufPtr);
        //Here it is possible to save the frame used in the vip_check_link routine as a
        // new stored offset image used in Preview Mode only
        //WORD* pxPtr = (WORD*)CheckLinkData.ChkBufPtr;
        //int result = vip_put_image(this->modeNumber, VIP_OFFSET_IMAGE, this->sensorWidth, this->sensorHeight, pxPtr);
    }

    //
    updateVarianState(0);
    
    return;
}

/* Function to obtain error message from error code
   Copied from Varian Sample Code */
void varian::GetCPError(int ErrorCode, char* ErrorMsg)
{
    static int HCPErrorCount = 0;
    if (!HCPErrorCount)
    {
        int i = 0;
        for (i = 0; i < MAX_HCP_ERROR_CNT; i++)
        {
            if (!strncmp(HcpErrStrList[i], "UUU", MIN_STR)) break;
        }
        if (i < MAX_HCP_ERROR_CNT)
            HCPErrorCount = i;
        if (HCPErrorCount <= 0)
        {
            if (ErrorMsg) strncpy(ErrorMsg, "Error at Compile Time", MAX_STR);
            return;
        }
    }

    if (ErrorCode < 0 || ErrorCode >= HCP_MAX_ERR_CODE)
    {
        if (ErrorCode == -1) ErrorCode = 3;
        else if (ErrorCode == 0x4000) ErrorCode = 5;
        else if (ErrorCode == 0x8000) ErrorCode = 6;
        else if (ErrorCode > 3500 && ErrorCode < 3600) ErrorCode -= 3492;
        else if (ErrorCode > 3299 && ErrorCode < 3314) ErrorCode -= 3283;
        else if (ErrorCode > 3399 && ErrorCode < 3419) ErrorCode -= 3367;
        else if (ErrorCode > 3429 && ErrorCode < 3435) ErrorCode -= 3378;
    }

    if (ErrorCode >= 0 && ErrorCode < HCPErrorCount)
    {
        if (ErrorMsg) _snprintf(ErrorMsg, MAX_STR, "%s; code=%d",
                                    HcpErrStrList[ErrorCode], ErrorCode);
    }
    else
    {
        if (ErrorMsg) _snprintf(ErrorMsg, MAX_STR, "Unknown Error; code=%d",
                                    ErrorCode);
    }
}

/* Function to obtain frame name from code found in HcpErrors.h
    need to find a better way to do this.
    Frame name is later stored in image metadata via the AreaDetector
    Attributes function.
*/
void varian::getFrameName(int frameCode, char* frameName)
{
    //printf("/n attempting to find framename for frame code %d %i 0x%.4X", frameCode, frameCode, frameCode);
    if (frameCode == 0xF4000010)
        _snprintf(frameName, MAX_STR,
                  "PREPULSE(Exposed-StoredOffset)/StoredGainImage");
    else if (frameCode == 0xF4000011) {
        //this is the "corrected image"
        _snprintf(frameName, MAX_STR, "(Exposed-PostOffset)/StoredGainImage");
    } else if (frameCode == 0xF400001F) {
        //this is the exposed ("raw") image
        _snprintf(frameName, MAX_STR, "ExposedImageNoCorrections");
    } else if (frameCode == 0)
        _snprintf(frameName, MAX_STR, "RawUncorrected");
    else if (frameCode == 1)
        _snprintf(frameName, MAX_STR, "StoredOffsetImage");
    else if (frameCode == 2)
        _snprintf(frameName, MAX_STR, "StoredGainImage");
    else if (frameCode == 3)
        _snprintf(frameName, MAX_STR, "BaseDefectImage");
    else if (frameCode == 4)
        _snprintf(frameName, MAX_STR, "AuxDefectImage");
    else if (frameCode == 5)
        _snprintf(frameName, MAX_STR, "VIPTestImage");
    else if (frameCode == 6)
        _snprintf(frameName, MAX_STR, "VIPReceptorTestImage");
    else if (frameCode == 7)
        _snprintf(frameName, MAX_STR, "VIPReceptorTestImageOff");
    else if (frameCode == 8)
        _snprintf(frameName, MAX_STR, "AnalogOffsetImage");
    else if (frameCode == 9)
        _snprintf(frameName, MAX_STR,
                  "(Exposed-StoredOffset)/StoredGainImage");
    else if (frameCode == 0xF4000021)
        _snprintf(frameName, MAX_STR, "PostOffsetImage");
}

/* Extract a specific correction frame for a given mode type, then
  "send to EPICS" via array callback (in sendFrameToEpics function)  */
void varian::extractFrame(int varianImageType)
{
    static char *functionName = "sendFrameToEpics";
    int result;
    USHORT *pFrame = new USHORT[this->sensorSize];

    result = vip_get_image(this->modeNumber, varianImageType,
                           this->sensorWidth, this->sensorHeight, pFrame);
    printAsynMessage("vip_get_image()", functionName, result);
    if (result != HCP_NO_ERR)
        return;

    sendFrameToEpics(pFrame, varianImageType);
}

/* "Send" a frame to EPICS via array callback and
   handle the EPICS AD image counters  */
void varian::sendFrameToEpics(USHORT *pFrame, int varianImageType)
{
    static char *functionName = "sendFrameToEpics";
    int arrayCallbacks, imageCounter, numImagesCounter;
    epicsTimeStamp startTime;

    pImage = this->pNDArrayPool->alloc(2, dims, NDUInt16, 0, NULL);
    pImage->pData = (USHORT *)pFrame;

    /* Update the frame counter */
    getIntegerParam(NDArrayCallbacks, &arrayCallbacks);
    if (arrayCallbacks) {
        getIntegerParam(NDArrayCounter, &imageCounter);
        getIntegerParam(ADNumImagesCounter, &numImagesCounter);
        imageCounter++;
        numImagesCounter++;
        setIntegerParam(NDArrayCounter, imageCounter);
        setIntegerParam(ADNumImagesCounter, numImagesCounter);

        pImage->uniqueId = imageCounter;
        pImage->timeStamp = startTime.secPastEpoch + startTime.nsec / 1.e9;

        /* here is where to update the image attributes that will later show up as image metadata "tags" */
        {
            // Current frame info
            UQueryProgInfo  QueryProgressInfoData;
            // Clear structure and set the structure size
            memset(&QueryProgressInfoData.qpitemps, 0,
                   sizeof(SQueryProgInfoFrame));
            QueryProgressInfoData.qpitemps.StructSize = sizeof(
                        SQueryProgInfoFrame);
            // FrameType and ReadyForExposure variables do not function -VCP bug?
            int result = vip_query_prog_info(HCP_U_QPIFRAME,
                                             &QueryProgressInfoData);

            // If call was successful print out the frame data
            // this data only applies to most recently acquired frame- see page 85
            // of the VCP manual
            if (result == HCP_NO_ERR)
            {   
                asynPrint(pasynUserSelf, ASYN_TRACE_FLOW,
                  "%s:%s: Saved frame info: AcqTim=%d ExposeStat=0x%.4X\n",
                  driverName, functionName, QueryProgressInfoData.qpiframe.AcqTime,
                       QueryProgressInfoData.qpiframe.Exposed);
            }

            // qpiframe.Exposed -enum
            // 0 idle frame, exposure enabled
            // 1 offset frame
            // 2 exposed frame
            // 3 idle frame, exposure disabled
            // 4 offset frame (alternate state) (whatever that means)
            // 5 "other"
            /* This is useless
            if (QueryProgressInfoData.qpiframe.Exposed & 0x0000)
                printf("Idle Frame, exposure enabled");
            if (QueryProgressInfoData.qpiframe.Exposed & 0x2000)
                printf("Offset frame");
            if (QueryProgressInfoData.qpiframe.Exposed & 0x8000)
                printf("Idle Frame, exposure enabled");
            if (QueryProgressInfoData.qpiframe.Exposed & 0x4000)
                printf("Idle Frame, exposure disabled");
            if (QueryProgressInfoData.qpiframe.Exposed & 0x6000)
                printf("Offset Frame, alternate state");
            */
            // Example code uses the following variants??
            // VIP_CURRENT_IMG_0 // VIP_CURRENT_IMG_1  // VIP_CURRENT_IMG_2
            // VIP_CURRENT_IMG_RAW // VIP_OFFSET_IMG_0 // VIP_OFFSET_IMG_1 // VIP_OFFSET_IMG_AV

            getAttributes(pImage->pAttributeList);

            // Acquisition time in milliseconds from 3024m internal clock
            pImage->pAttributeList->add("VarianAcquisitionTimeMillisec",
                                        "Varian Acquisition Time ms", NDAttrInt32,
                                        &QueryProgressInfoData.qpiframe.AcqTime);

            // string describing current mode
            pImage->pAttributeList->add("VarianModeDescription",
                                        "Varian Mode Description", NDAttrString, &gModeInfo.ModeDescription);

            // Double mode on or off?
            int _doubleMode;
            getIntegerParam(DoubleMode, &_doubleMode);
            pImage->pAttributeList->add("VarianDoubleMode", "Double Exposure",
                                        NDAttrInt32, &_doubleMode);

            // string describing image type
            char _frameName[MAX_STR] = "";
            getFrameName(varianImageType, _frameName);
            pImage->pAttributeList->add("VarianFrameType", "Varian Frame Type",
                                        NDAttrString, _frameName);

            //string showing time last calibrated
            char *_timeLastCal = asctime(localtime(&lastCalTime));
            pImage->pAttributeList->add("VarianCalibrationTime",
                                        "Time Last Calibrated", NDAttrString, _timeLastCal);
        }

        /* Call the NDArray callback */
        /* Must release the lock here, or we can get into a deadlock, because we can
        /* block on the plugin lock, and the plugin can be calling us */
        this->unlock();
        doCallbacksGenericPointer(pImage, NDArrayData, 0);
        this->lock();
    }
    
    asynPrintIO(this->pasynUserSelf, ASYN_TRACEIO_DRIVER,
                (const char *)this->pImage->pData, this->pImage->dataSize,
                "%s:%s", driverName, functionName);
    
    pImage->release();
    pImage = NULL;
}

/* From asynPortDriver: Connects driver to device */
asynStatus varian::connect( asynUser* pasynUser ) {
    return connectCamera();
}

asynStatus varian::connectCamera()
{
    static char *functionName = "connectCamera";
    int result = HCP_NO_ERR;
    int status = asynSuccess;

    //update state to "connecting"
    updateVarianState(6);

    this->acqInProg = FALSE;
    // open link using this->gszIniPath from varianConfig command
    {
        SOpenReceptorLink  orl;
        memset(&orl, 0, sizeof(SOpenReceptorLink));
        orl.StructSize = sizeof(SOpenReceptorLink);
        // If you want to enable debugging at boot
        orl.DebugMode = HCP_DBG_ON_FLSH;
        orl.MaxModesPerRcpt = 30;

        /* these settings have no effect with 3024m */
        /*
        orl.SubModeBinX = 2;
        orl.SubModeBinY = 2;
        */

        // callback function to be called when connection to receptor is lost
        // very serious error which should only occur if cable is disconnected,
        // power is lost, or somehow the driver on the machine gets unloaded
        orl.FgCallbackPtr  = connectionLostCallbackC;
        orl.FgCallbackFlag = HCP_FG_CALLBACK_FLAG;

        strcpy_s(orl.RecDirPath, this->gszIniPath);
        printf("\nOpening link to %s\n", this->gszIniPath);
        result = vip_open_receptor_link(&orl);
    }
    
    printAsynMessage("vip_open_receptor_link()",
                         functionName, result);
    /*
    if (result != HCP_NO_ERR)
    {
        //ignore missing corrections errors
        if (result != HCP_OFST_ERR || HCP_GAIN_ERR || HCP_DFCT_ERR )
        {   printf("Error opening receptor link \n");
            return asynError;
        } else {
            printf("Corrections files are missing \n");
        }
    }
    */
    
    Sleep(200); //Necessary?

    // need to select mode to init mode settings??
    //result = vip_select_mode(this->modeNumber);
    //printAsynMessage("vip_select_mode()", functionName, result);

    if (this->checkLinkOnConnect == true)
        checkLink();

    //Prepare DeviceInfo structure for vip_get_sys_info call
    SDeviceInfo1 DeviceInfo;
    memset(&DeviceInfo, 0, sizeof(DeviceInfo));
    DeviceInfo.StructSize = sizeof(DeviceInfo);
    DeviceInfo.StructType = 1;

    //Prepare SSysInfo structure for vip_get_sys_info call
    SSysInfo SystemInfo;
    memset(&SystemInfo, 0, sizeof(SystemInfo));
    SystemInfo.StructSize = sizeof(SSysInfo);
    SystemInfo.DeviceInfoPtr = &DeviceInfo;

    // Perform the get sys info call
    result = vip_get_sys_info(&SystemInfo);
    printAsynMessage("vip_get_sys_info()", functionName, result);

    // On the 3024M, there is no support for binning or ROI
    //   so the size/dimensions of the frame never change
    this->sensorWidth  = SystemInfo.MxColsPerFrame;
    this->sensorHeight = SystemInfo.MxLinesPerFrame;
    this->sensorSize   = SystemInfo.MxLinesPerFrame *
                         SystemInfo.MxColsPerFrame;
    this->maxFrameSize = 16 * SystemInfo.MxLinesPerFrame *
                         SystemInfo.MxColsPerFrame;
    this->maxPixelValue = SystemInfo.MxPixelValue;
    this->numModes     = SystemInfo.NumModes;
    this->dims[0]      = this->sensorWidth;
    this->dims[1]      = this->sensorHeight;

    UQueryProgInfo qpiData;
    memset(&qpiData, 0, sizeof(qpiData));
    qpiData.qpircpt.StructSize = sizeof(SQueryProgInfoRcpt);

    // init prog info settings
    result = vip_query_prog_info(HCP_U_QPIRCPT | HCP_U_QPI_CRNT_DIAG_DATA,
                                 &qpiData);
    printAsynMessage("vip_query_prog_info()", functionName,
                     result);

    char boardSN[20];
    sprintf (boardSN, "%.2X%.2X%.2X",
             qpiData.qpircpt.BoardSNbr[2],
             qpiData.qpircpt.BoardSNbr[1],
             qpiData.qpircpt.BoardSNbr[0]);

    // Most of these are constant and thus can be set once upon initial connection
    status  = setIntegerParam(VarianMode, this->modeNumber);
    status |= setIntegerParam(NumberOfModes, this->numModes);
    status |= setIntegerParam(FirmwareVersion, qpiData.qpircpt.FwVersion);
    status |= setStringParam(ReceptorConfigPath, this->gszIniPath);
    status |= setStringParam(MacAddress, DeviceInfo.MacAddress);
    status |= setStringParam(IpAddress, DeviceInfo.IpAddress);
    status |= setStringParam(BoardSerialNumber, boardSN);   

    // set constant AreaDetector and NDArray parameters
    status |= setStringParam(ADManufacturer, "Varian");
    status |= setStringParam(ADModel, SystemInfo.SysDescription);
    status |= setIntegerParam(ADSizeX, this->sensorWidth);
    status |= setIntegerParam(ADSizeY, this->sensorHeight);
    status |= setIntegerParam(ADMaxSizeX, this->sensorWidth);
    status |= setIntegerParam(ADMaxSizeY, this->sensorHeight);
    status |= setIntegerParam(NDDataType, NDUInt16);
    status |= setIntegerParam(NDColorMode, NDColorModeMono);
    status |= setIntegerParam(NDArraySizeX,
                              this->sensorWidth); //in pixels
    status |= setIntegerParam(NDArraySizeY,
                              this->sensorHeight); // in pixels
    status |= setIntegerParam(NDArraySize,
                              sizeof(USHORT) * this->sensorSize); // in bytes

    status = pasynManager->exceptionConnect(this->pasynUserSelf);
    if (status) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                  "%s:%s: error calling pasynManager->exceptionConnect, error=%s\n",
                  driverName, functionName, pasynUserSelf->errorMessage);
        return asynError;
    }

    /* do we need this */
    vip_reset_state();

    setIntegerParam(ADStatus, ADStatusIdle);
    asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,
              "%s:%s: Camera connected\n", driverName, functionName);
    
    //update state to "ready"
    updateVarianState(0);

    result = vip_dcds_enable(TRUE);
    if (result != HCP_NO_ERR)
    {
        printAsynMessage("Error calling  vip_dcds_enable(TRUE)", functionName, result);
    }

    //update mode settings 
    int currModeNum;
    vip_get_current_mode(&currModeNum);
    //changeMode(1); //for now just reset to mode number 0 whenever connecting
    changeMode(currModeNum);
    setIntegerParam(VarianMode, currModeNum);
    callParamCallbacks();

    return asynSuccess;
}

/* From asynPortDriver: Disconnects driver from device */
asynStatus varian::disconnect( asynUser* pasynUser ) {
    return disconnectCamera();
}

asynStatus varian::disconnectCamera()
{
    int status;
    static char *functionName = "disconnectCamera";

    status = vip_close_link();
    printAsynMessage("vip_close_link()", functionName, status);

    Sleep(500);

    if (status) {
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                  "%s:%s: unable to close camera \n",
                  driverName, functionName);
        return asynError;
    }
    // if vip_close_link was successful we are now disconnected
    updateVarianState(7);

    /* We've disconnected the camera. Signal to asynManager that we are disconnected. */
    status = pasynManager->exceptionDisconnect(this->pasynUserSelf);
    if (status) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                  "%s:%s: error calling pasynManager->exceptionDisconnect, error=%s\n",
                  driverName, functionName, pasynUserSelf->errorMessage);
    }
    
    setIntegerParam(ADStatus, ADStatusDisconnected);

    asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,
              "%s:%s: Camera disconnected\n",
              driverName, functionName);
    return ((asynStatus)status);
}

/* Called by VirtualCP if we lose connection to the receptor
   Attempts to restore connection */
void varian::connectionLostCallback()
{
    printf("\n\n******");
    printf("\n****** COMMUNICATION ERROR");
    printf("\n****** Link to Receptor was lost");
    printf("\n****** Attempting to reconnect");
    printf("\n******\n\n");

    int result = vip_close_link();
    vip_set_debug(HCP_DBG_OFF);
    setIntegerParam(ADStatus, ADStatusDisconnected);
    updateVarianState(7);

    /* manual says the callback must be designed to return in <100 msec,
        so we can not reconnect in this function */
    return;
}

/* Attempts to cancel by sending HS_CANCEL signal to panel, returns TRUE if
    succesful, FALSE if cancellation fails. Currently not used. */
bool varian::DoCancel() {
    static char *functionName = "DoCancel";
    int result = doHandSwitch(HS_CANCEL);

    printAsynMessage("cancel request: doHandSwitch(HS_CANCEL)",
                     functionName, result);

    if (result == 0)
        doHandSwitch(HS_STANDBY);

    return result == 0;
}


// either HS_STANDBY or HS_ACTIVE, handle chores
int varian::doHandSwitch(int newHSState){
    static char *functionName = "doHandSwitch";
    
    int result = vip_io_enable(newHSState);
    printAsynMessage("cancel request: vip_io_enable(HS_CANCEL)",
                     functionName, result);

    if (result != HCP_NO_ERR)
        return result; //error
    
    setIntegerParam(HandswitchState, newHSState);
    callParamCallbacks();

    return result;
}

/* We have two different config folders from Varian, and may want to switch between
   them frequently during use. One is devoted to radiography, the other fluoro.  */
void varian::changeConfig(int configNum)
{
    static char *functionName = "changeConfig";
    asynStatus status;
    printAsynMessage("Attempting to reload link with a different config folder",
                     functionName, VIP_NO_ERR);
    
    this->lock();
    disconnectCamera();
    if (configNum == 0)
        this->gszIniPath = this->gszIniPathVarianRad;
    else if (configNum == 1)
        this->gszIniPath = this->gszIniPathVarianFluoro;
    else if (configNum == 2)
        this->gszIniPath = this->gszIniPathCustom1;
    else if (configNum == 3)
        this->gszIniPath = this->gszIniPathCustom2;


    status = connectCamera();

    if (status) {
        printf("%s:%s: cannot connect to camera, manually connect when available.\n",
               driverName, functionName);
        return;
    }
    printf("\nSuccessfully switched configuration!");
    this->currConfigNum = configNum;

    //change mode to update mode dependent parameters
    int _currentMode;
    if (configNum == 1)
        getIntegerParam(VarianModeFluoro, &_currentMode);
    else {
        getIntegerParam(VarianMode, &_currentMode);
        //override default mode of 0 in rad mode, to be 2 sec mode
        _currentMode = 1;
    }
    changeMode(_currentMode);
    this->unlock();
}

/* Change VirtualCP debug mode */
void varian::changeDebug(int newDebugMode)
{
    static char *functionName = "changeDebug";
    int result;
    int status = asynSuccess;

    // can be set to 4 possible values (0-3)
    // 0 HCP_DBG_OFF - disable debugging
    // 1 HCP_DBG_ON - write debug output to file when debug is disabled
    // 2 HCP_DBG_ON_FLSH - write debug output to file continuously
    // 3 HCP_DBG_ON_DLG - same as 1 but with popup dialog(Win MFC only)

    // If compiling with Windows MFC _MFC_VER should be defined
#ifndef _MFC_VER
    if (newDebugMode == HCP_DBG_ON_DLG)
        newDebugMode = HCP_DBG_OFF;
#endif
    result = vip_set_debug(newDebugMode);
    printAsynMessage("vip_set_debug()", functionName, result);
}

// need to refactor: if corrections available, enable them.
// if not disable them to prevent errors 
/* Change current mode correction settings */
void varian::changeCorrections()
{
    static char *functionName = "changeCorrections";
    int result, _offset, _gain, _defect;

    // Get values
    getIntegerParam(OffsetCorrection, &_offset);
    getIntegerParam(GainCorrection,   &_gain);
    getIntegerParam(DefectCorrection, &_defect);
    // line corrections not implemented by Varian
    //getIntegerParam(LineCorrection, &_line);

    // Create SCorrections structure
    SCorrections CorrectionsData;
    memset(&CorrectionsData, 0, sizeof(SCorrections));
    CorrectionsData.StructSize = sizeof(SCorrections);

    CorrectionsData.Ofst = _offset;
    CorrectionsData.Gain = _gain;
    CorrectionsData.Dfct = _defect;
    // line corrections not implemented by Varian
    //CorrectionsData.Line = _line;

    // set correction settings
    result = vip_set_correction_settings(&CorrectionsData);
    printAsynMessage("vip_set_correction_settings()", functionName,
                     result);

    if (result != HCP_NO_ERR){
        switch (result)
        {
            case HCP_OFST_ERR:
                printf("Requested corrections not available: offset file missing\n");
                break;
            case HCP_GAIN_ERR:
                printf("Requested corrections not available: gain file missing\n");
                CorrectionsData.Ofst = TRUE;
                break;
            case HCP_DFCT_ERR:
                printf("Requested corrections not available: defect file missing\n");
                CorrectionsData.Ofst = TRUE;
                CorrectionsData.Gain = TRUE;
                break;
            default:
                break;
        }
        result = vip_set_correction_settings(&CorrectionsData);
    }
    // readback actual settings
    result = vip_get_correction_settings(&CorrectionsData);
    printAsynMessage("vip_get_correction_settings()", functionName,
                     result);

    setIntegerParam(OffsetCorrection, CorrectionsData.Ofst);
    setIntegerParam(GainCorrection,   CorrectionsData.Gain);
    setIntegerParam(DefectCorrection, CorrectionsData.Dfct);

    callParamCallbacks();
}

/* Change mode and update mode dependent parameters */
void varian::changeMode(int newModeNumber)
{
    static char *functionName = "changeMode";
    int result, currModeNum;
    int status = asynSuccess;

    if( this->modeNumber != newModeNumber )
    {
        result = vip_select_mode(newModeNumber);
        printAsynMessage("vip_select_mode()", functionName, result);
    }
    // todo : only ignore missing corrections
    //if (result != HCP_NO_ERR && result != (HCP_OFST_ERR || HCP_GAIN_ERR || HCP_DFCT_ERR ))
     //   return ;

    // Verify mode number and set it to local memory/epics
    vip_get_current_mode(&currModeNum);
    this->modeNumber = currModeNum;
    setIntegerParam(VarianMode, currModeNum);

    // Refresh persistent mode info struct
    result = vip_get_mode_info(currModeNum, &gModeInfo);
    printAsynMessage("vip_get_mode_info()", functionName, result);

    this->radMode = gModeInfo.AcqType; // 1 is rad mode, 0 is fluoro
    //status |= setIntegerParam(VarianRadOrFluoro, ModeInfo.AcqType);
    status |= setDoubleParam(VarianFrameRate, gModeInfo.FrameRate);
    /* todo: new params to add
    status |= setDoubleParam(VarianMaxFrameRate, gModeInfo.MxAllowedFrameRate);
    status |= setIntegerParam(ModeAcqFrameCount, gModeInfo.AcqFrmCount);
    status |= setIntegerParam(ModeXBinning, gModeInfo.ColsPerPixel);
    status |= setIntegerParam(ModeYBinning, gModeInfo.LinesPerPixel);
    status |= setIntegerParam(UserSync, gModeInfo.UserSync);
    status |= setIntegerParam(DCDSEnable, gModeInfo.DcdsEnable)
    */
    //printf("\nacqfrmcount: %d, calframecount: %d\n", gModeInfo.AcqFrmCount, gModeInfo.CalFrmCount);
    printf("DCDSEnable is  %d\n", gModeInfo.DcdsEnable);

    status |= setStringParam(VarianModeDescription,
                             gModeInfo.ModeDescription);

    //printf("MammoModeTypeFlag is d %d\n", gModeInfo.MammoModeTypeFlag);
    this->varianAcqType = gModeInfo.MammoModeTypeFlag;
    status |= setIntegerParam(VarianAcquisitionType, gModeInfo.MammoModeTypeFlag);
    // 0 is standard radiography with post offset
    // 2 is prepulse
    // 3 is prepulse
    // 4 is fluoro
 
    callParamCallbacks();
    /* UserSync setting seems to have no effect on the 3024m
    if (gModeInfo.UserSync == true)
        printf("\n user sync is on");
    if (this->radMode == 0){
        //we are in fluoro mode, try and turn user sync on
        printf("attempting to set user sync to true");
        result = vip_set_user_sync(this->modeNumber, TRUE);
        if (result != HCP_NO_ERR)
        {
            printAsynMessage("Error calling  vip_set_user_sync()", functionName, result);
            return;
        }
    }*

    /* this actually works, and disables DCDS until a calibration is performed (or until it is re-enabled)
    result = vip_dcds_enable(FALSE);
    if (result != HCP_NO_ERR)
    {
        printAsynMessage("Error calling  vip_dcds_enable(FALSE)", functionName, result);
        return;
    }*/

    /* I believe these hwConfig settings are of no use to 3024m
    SHwConfig hwConfig;
    memset(&hwConfig, 0, sizeof(SHwConfig));
    hwConfig.StructSize = sizeof(SHwConfig);
    //hwConfig.FrmIntrptSrc = 0;
    //vip_get_hw_config(&hwConfig);

    //vip_set_hw_config(&hwConfig);
    vip_get_hw_config(&hwConfig);
    printf("frameinterruptSrc: %d\n", hwConfig.FrmIntrptSrc);
    printf("timeparam: %d\n", hwConfig.TimeParam1);
    */

    /* no effect with 3024m
    result=  vip_signal_frame_start();
    if (result != HCP_NO_ERR)
        printf("vip_signal_frame_start");
    */

    // Get info about current mode calibration files
    SCalInfo CalInfo;
    memset(&CalInfo, 0, sizeof(SCalInfo));
    CalInfo.StructSize = sizeof(SCalInfo);

    result = vip_get_cal_info(currModeNum, &CalInfo);
    printAsynMessage("vip_get_cal_info()", functionName, result);

    setDoubleParam(OffsetMedian, CalInfo.OfstMedian);
    setDoubleParam(OffsetStdDev, CalInfo.OfstStdDev);
    setDoubleParam(GainMedian,   CalInfo.GainMedian);
    setDoubleParam(GainStdDev,   CalInfo.GainStdDev);
    setDoubleParam(GainScaling,  CalInfo.GainScaling);

    // need to convert long seconds since epoch from CalInfo
    //  to a readable string
    lastCalTime = static_cast<time_t> (CalInfo.Time);
    struct tm * timeinfo;
    timeinfo = localtime (&lastCalTime);
    status |= setStringParam(TimeLastCalibrated, asctime(timeinfo));

    // get current correction settings for the "corrected" frame
    SCorrections CorrectionsData;
    memset(&CorrectionsData, 0, sizeof(SCorrections));
    CorrectionsData.StructSize = sizeof(SCorrections);

    result = vip_get_correction_settings(&CorrectionsData);
    printAsynMessage("vip_get_correction_settings()", functionName,
                     result);

    setIntegerParam(OffsetCorrection, CorrectionsData.Ofst);
    setIntegerParam(GainCorrection,   CorrectionsData.Gain);
    setIntegerParam(DefectCorrection, CorrectionsData.Dfct);

    /* Although it accepts this call and returns no error, this has no effect on rad imaging speed
       or fluoro imaging speed. Likely because they mention that the 3024m is a "fixed frame rate panel" */
    /*
    result = vip_set_frame_rate(this->modeNumber, 1.0 );
    printf("attempting to set frame rate to %f result: %d", fr, result);
    if (result != HCP_NO_ERR)
    {
        printAsynMessage("Error calling  vip_set_frame_rate()", functionName, result);
        return;
    }
    */

    callParamCallbacks();
}

/* This function should quickly update all parameters from the panel,
   especially parameters which are subject to change regularly.
   This is most important for us to see the RFP, complete, numframes, numpulses
   signals from the panel.
   Temperatures are not very crucial for us to monitor, since we rarely see
   them fluctuate, and our panel is in an exposed open-air configuration.
   parameters which are fixed should be set at connect.
   parameters which only change with mode are handled in changeMode(). */
asynStatus varian::readParameters()
{
    static char *functionName = "readParameters";
    int status = asynSuccess;
    int result;

    //Use SQueryProgInfo to obtain current status information
    UQueryProgInfo CurrentStatus;
    memset(&CurrentStatus, 0, sizeof(UQueryProgInfo));
    CurrentStatus.qpi.StructSize = sizeof(SQueryProgInfoRcpt);
    CurrentStatus.qpitemps.StructSize = sizeof(SQueryProgInfoTemps);

    // Perform the query progress info call, only relevant during rad
    // (in fluoro mode these signals are not used)
    if (this->radMode) {
        result = vip_query_prog_info(HCP_U_QPI, &CurrentStatus);
        printAsynMessage("vip_query_prog_info(HCP_U_QPI)", functionName, result);
        if (result == VIP_NO_ERR) {
            setIntegerParam(NumFrames, CurrentStatus.qpi.NumFrames);
            setIntegerParam(Complete, CurrentStatus.qpi.Complete);
            setIntegerParam(NumPulses, CurrentStatus.qpi.NumPulses);
            setIntegerParam(ReadyForPulse, CurrentStatus.qpi.ReadyForPulse);
        }
    }

    /* this only works if an image was recently acquired */
    /* can change it to be polling method */
    // max values(C)- 72,51,72,72,72,72
    result = vip_query_prog_info(HCP_U_QPITEMPS |
                                 HCP_U_QPI_CRNT_DIAG_DATA, &CurrentStatus);
    //if (result)
    //    printAsynMessage("Error calling vip_query_prog_info(HCP_U_QPITEMPS)", functionName, result);
    if (result == VIP_NO_ERR) {
        setDoubleParam(Temp1, CurrentStatus.qpitemps.Celsius[0]);
        setDoubleParam(Temp2, CurrentStatus.qpitemps.Celsius[1]);
        setDoubleParam(Temp3, CurrentStatus.qpitemps.Celsius[2]);
        setDoubleParam(Temp4, CurrentStatus.qpitemps.Celsius[3]);
        setDoubleParam(Temp5, CurrentStatus.qpitemps.Celsius[4]);
        setDoubleParam(Temp6, CurrentStatus.qpitemps.Celsius[5]);
    }

    callParamCallbacks();
    return asynSuccess;
}

/* Prints asyn message
   If vcpResult was  an error, print error code with ASYN_TRACE_ERROR.
   Else print as ASYN_TRACE_FLOW */
void varian::printAsynMessage(char* message, char* functionName,
                              int vcpResult) {
    if (vcpResult) {
        // Translate return code into a printable message
        char errorMessage[MAX_STR] = "";
        GetCPError(vcpResult, errorMessage);
        //because there was an error we use ASYN_TRACE_ERROR
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                  "%s:%s: %s : %s \n",
                  driverName, functionName, message,  errorMessage);
    } else {
        // no VCP error, use ASYN_TRACE_FLOW
        asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, "%s:%s: %s \n",
                  driverName, functionName, message);
    }
}

/* This is called from the other various methods of Querying the panel.
   Retrieves the current status of the virtual CP system and
   populates the qpi member of the UQueryProgInfo structure
   It updates relevant EPICS PVs regarding panel status */
int varian::QueryProgress(UQueryProgInfo &CurrentStatus)
{
    // Store the previous state of the status flags so we can limit
    // EPICS param updates to only when a status flag changes
    UQueryProgInfo prevStatus = CurrentStatus;
    memset(&CurrentStatus, 0, sizeof(SQueryProgInfo));
    CurrentStatus.qpi.StructSize = sizeof(SQueryProgInfo);
    int result = vip_query_prog_info(HCP_U_QPI, &CurrentStatus);

    // Update epics status parameters if they change
    if (result == HCP_NO_ERR)
    {
        if ((prevStatus.qpi.NumFrames != CurrentStatus.qpi.NumFrames)
                || (prevStatus.qpi.Complete != CurrentStatus.qpi.Complete)
                || (prevStatus.qpi.NumPulses != CurrentStatus.qpi.NumPulses)
                || (prevStatus.qpi.ReadyForPulse != CurrentStatus.qpi.ReadyForPulse)
                || (prevStatus.qpi.Prepare != CurrentStatus.qpi.Prepare))
        {
            setIntegerParam(NumFrames, CurrentStatus.qpi.NumFrames);
            setIntegerParam(Complete, CurrentStatus.qpi.Complete);
            setIntegerParam(NumPulses, CurrentStatus.qpi.NumPulses);
            setIntegerParam(ReadyForPulse, CurrentStatus.qpi.ReadyForPulse);

            printf("QueryProgress: NumFrames=%d Complete=%d NumPulses=%d RFP=%d Prep=%d\n"
                        , CurrentStatus.qpi.NumFrames
                        , CurrentStatus.qpi.Complete
                        , CurrentStatus.qpi.NumPulses
                        , CurrentStatus.qpi.ReadyForPulse
                        , CurrentStatus.qpi.Prepare);
        }

    }
    return result;
}

/* Copied from varian example- wait until a certain number of frames has been reached */
int varian::QueryWaitOnNumFrames(UQueryProgInfo &CurrentStatus,
                                 int TargetNumFrames,
                                 int framesPerExposed,
                                 int TimeoutMSec,
                                 bool *CancelFlagPtr)
{
    if (CancelFlagPtr && (*CancelFlagPtr != 0))
        printf("Waiting for Cancellation result (or NumFrames == %d)\n",
               TargetNumFrames / framesPerExposed);
    //else
    //    printf("Waiting for NumFrames == %d...", TargetNumFrames/framesPerExposed);

    int TotalMsec = 0;
    // Get current status
    int result = HCP_NO_ERR;

    result = QueryProgress(CurrentStatus);

    // If call was successful loop until the number of frames matches
    // the target number of frames
    while ((result == HCP_NO_ERR) &&
            (CurrentStatus.qpi.NumFrames < TargetNumFrames))
    {
        // Sleep
        Sleep(QUERY_WAIT_INTERVAL);
        //  Check for timeout and break out of loop if timeout occurs
        if (TimeoutMSec != TIMEOUT_UNLIMITED)
        {
            TotalMsec += QUERY_WAIT_INTERVAL;
            if (TotalMsec >= TimeoutMSec)
            {
                printf("*** NumFrames=%d: target of %d not reached within %d ms\n",
                       CurrentStatus.qpi.NumFrames, TargetNumFrames, TimeoutMSec);
                result = HCP_TIMEOUT;
                break;
            }
        }
        // If acquire is set to 0 it means user is attempting to cancel
        /* this doesn't apply during calibration routines
        getIntegerParam(ADAcquire, &_acquire);
        if (_acquire == 0)
        {
            if(CancelFlagPtr)
            {
                *CancelFlagPtr = true;
                result = 0;
                break;
            } else {
                printf("\n*** Cancel request cannot be accepted at this time\n\n");
            }
        }
        */

        result = QueryProgress(CurrentStatus);
    }
    return result;
}

/* Copied from varian example- wait until panel is ready for pulse */
int varian::QueryWaitOnReadyForPulse(UQueryProgInfo &CurrentStatus,
                                     int TargetState,
                                     int TimeoutMSec,
                                     bool* CancelFlagPtr,
                                     bool bVerbose)
{

    int result = HCP_NO_ERR;
    int TotalMsec = 0;

    result = QueryProgress(CurrentStatus);

    // If call was successful continue querying status until
    //  the ReadyForPulse flag becomes true
    while ((result == HCP_NO_ERR || result == HCP_DEVICE_BUSY) &&
            (CurrentStatus.qpi.ReadyForPulse != TargetState))
    {
        // Sleep
        Sleep(QUERY_WAIT_INTERVAL);
        //  Check for timeout and break out of loop if timeout occurs
        if (TimeoutMSec != TIMEOUT_UNLIMITED)
        {
            TotalMsec += QUERY_WAIT_INTERVAL;
            if (TotalMsec >= TimeoutMSec)
            {
                if (bVerbose)
                    printf("*** ReadyForPulse=%d: target of %d not reached within %d ms\n",
                           CurrentStatus.qpi.ReadyForPulse, TargetState, TimeoutMSec);
                result = HCP_TIMEOUT;
                break;
            }
        }
        /*
        // If acquire is set to 0 it means user is attempting to cancel
        getIntegerParam(ADAcquire, &_acquire);
        if (_acquire == 0)
        {
            if(CancelFlagPtr)
            {
                *CancelFlagPtr = true;
                result = 0;
                break;
            } else {
                printf("\n*** Cancel request cannot be accepted at this time\n\n");
            }
        }
        */

        // No timeout or cancel request occured so continue querying the status
        QueryProgress(CurrentStatus);
    }
    return result;
}

int varian::QueryWaitOnNumPulses(UQueryProgInfo &CurrentStatus,
                                 int TargetNumberPulses,
                                 int TimeoutMSec,
                                 bool *CancelFlagPtr)
{
    static char *functionName = "QueryWaitOnNumPulses";

    asynPrint(pasynUserSelf, ASYN_TRACE_FLOW,
                      "%s::%s Waiting for NumPulses == %d",
                      driverName, functionName, TargetNumberPulses);

    int TotalMsec = 0;
    // Get current status
    int result = QueryProgress(CurrentStatus);

    // If call was successful loop until the number of pulses matches
    // the target number of pulses
    while ((result == HCP_NO_ERR) &&
            (CurrentStatus.qpi.NumPulses < TargetNumberPulses))
    {
        // Sleep
        Sleep(QUERY_WAIT_INTERVAL);
        //  Check for timeout and break out of loop if timeout occurs
        if (TimeoutMSec != TIMEOUT_UNLIMITED)
        {
            TotalMsec += QUERY_WAIT_INTERVAL;
            if (TotalMsec >= TimeoutMSec)
            {
                asynPrint(pasynUserSelf, ASYN_TRACE_FLOW,
                      "%s::%s NumPulses=%d: target of %d not reached within %d ms",
                      driverName, functionName, CurrentStatus.qpi.NumPulses, TargetNumberPulses, TimeoutMSec);
                result = HCP_TIMEOUT;
                break;
            }
        }
        // No timeout or cancel request occured so continue querying the status
        result = QueryProgress(CurrentStatus);
    }
    return result;
}

int varian::QueryWaitOnComplete(UQueryProgInfo &CurrentStatus,
                                int TimeoutMSec,
                                bool *CancelFlagPtr)
{
    int TotalMsec = 0;
    // Get the current status
    int result = QueryProgress(CurrentStatus);

    // If call was successful continue querying status until
    // the Complete flag becomes true
    while ((result == HCP_NO_ERR) && (CurrentStatus.qpi.Complete != TRUE))
    {
        Sleep(QUERY_WAIT_INTERVAL);
        //  Check for timeout and break out of loop if timeout occurs
        if (TimeoutMSec != TIMEOUT_UNLIMITED)
        {
            TotalMsec += QUERY_WAIT_INTERVAL;
            if (TotalMsec >= TimeoutMSec)
            {
                printf("*** Complete not detected within %d ms\n", TimeoutMSec);
                result = HCP_TIMEOUT;
                break;
            }
        }
        /*
        getIntegerParam(ADAcquire, &_acquire);
        if (_acquire == 0)
        {
            if(CancelFlagPtr)
            {
                *CancelFlagPtr = true;
                result = 0;
                break;
            }
            else
            {
                printf("\n*** Cancel request cannot be accepted at this time\n\n");
            }
        }*/

        // No timeout or cancel request occured so continue querying the status
        result = QueryProgress(CurrentStatus);
    }

    return result;
}

int varian::QueryWaitOnStandby(UQueryProgInfo &CurrentStatus,
                               int TimeoutMSec,
                               bool *CancelFlagPtr)
{
    DWORD startWaitTime = GetTickCount();
    int result = HCP_NO_ERR;
    // Get current status
    QueryProgress(CurrentStatus);

    // If call was successful continue querying status until
    // the ReadyForPulse flag becomes true
    while ((result == HCP_NO_ERR) &&
            ((CurrentStatus.qpi.NumFrames != 0) ||
             (CurrentStatus.qpi.ReadyForPulse != 0) ))
    {
        //  Check if the system fails to return to HS_STANDBY state
        //  within the requested interval.
        if (TimeoutMSec <= (int) (GetTickCount() - startWaitTime))
        {
            printf("*** TIMEOUT ***\n");
            result = HCP_TIMEOUT;
            break;
        }

        // No timeout or cancel request occured so continue querying the status
        QueryProgress(CurrentStatus);
    }
    return result;
}

void varian::StartTickCount()
{   
    this->gdwStartTickCount = GetTickCount();
}

void varian::EndTickCount(char* szTickCountMsg)
{   
    strcpy(this->gszTickCounts[this->gnTickCountIdx], szTickCountMsg);
    this->gdwTick[this->gnTickCountIdx] = GetTickCount() - this->gdwStartTickCount;
    this->gnTickCountIdx++;
}

void varian::ResetTickCounts()
{
    this->gnTickCountIdx = 0;
}

void varian::PrintTickCounts()
{
    static char *functionName = "PrintTickCounts";
    if ( this->gnTickCountIdx == 0)
        return;
    for ( int idx = 0; idx < this->gnTickCountIdx; idx++)
        asynPrint(pasynUserSelf, ASYN_TRACE_FLOW,
                      "%s::%s ---   %s = %d\n",
                      driverName, functionName, this->gszTickCounts[idx],
                      this->gdwTick[idx]);
}

/* No x-rays and thus no syncing is required for this calibration.*/
int varian::performRadAnalogOffsetCal()
{
    static char *functionName = "performRadAnalogOffsetCal";
    SAnalogOffsetParams aop;
    memset (&aop, 0, sizeof(SAnalogOffsetParams));
    aop.StructSize = sizeof(SAnalogOffsetParams);

    if (this->radMode)
        updateVarianState(3); // rad cal state
    else
        updateVarianState(4); // fluoro cal state

    int result = vip_get_analog_offset_params(this->modeNumber, &aop);
    printAsynMessage("vip_get_analog_offset_params()", functionName,
                     result);

    // these numbers should already be known from the panel config file
    // they can also be set here
    //aop.TargetValue=1000;
    //aop.Tolerance=50;
    //aop.MedianPercent=50;
    //aop.FracIterDelta=0.33;
    //aop.NumIterations=10;

    if (result == VIP_NO_ERR)
    {
        printf ("\n\nAnalog offset params are:-\nTargetValue=%d; Tolerance=%d;"
                "\nMedianPercent=%d; FracIterationDelta=%.3f; NumberIterations=%d",
                aop.TargetValue, aop.Tolerance,
                aop.MedianPercent, aop.FracIterDelta, aop.NumIterations);

        // start the analog offset cal
        result = vip_analog_offset_cal(this->modeNumber);
        printAsynMessage("vip_analog_offset_cal()", functionName, result);
    }

    if (result == VIP_NO_ERR)
    {
        UQueryProgInfo uq;
        memset(&uq, 0, sizeof(UQueryProgInfo));
        uq.qpi.StructSize = sizeof(SQueryProgInfo);

        printf("\n\nAnalog Offset Calibration in Progress");

        while (!uq.qpi.Complete && result == VIP_NO_ERR)
        {
            Sleep(1000);
            printf(".");
            result = vip_query_prog_info(HCP_U_QPI, &uq);
        }
    }
    
    updateVarianState(0);
    printf("\n\nAnalog Offset Calibration end");

    return result;
}

/* perform a fluoro acquisition. This was modified from fluorotest.cpp, which
    was supplied by Varian */
asynStatus varian::performFluoroAcquisition()
{
    static char *functionName = "performFluoroAcquisition";
    int numBuf = 10;
    int numFrm = 0;
    int result = VIP_NO_ERR;

    updateVarianState(2);
    /* settings seem to have no effect on 3024m
    SFluoroModePrms fluoroPrms;
    memset(&fluoroPrms, 0, sizeof(SFluoroModePrms));
    fluoroPrms.StructSize = sizeof(SFluoroModePrms);

    fluoroPrms.FrameX = this->sensorWidth;
    fluoroPrms.FrameY = this->sensorHeight;
    fluoroPrms.TrigSrc = 1;
    fluoroPrms.TrigMode = 1;
    fluoroPrms.UserSync = 1;

    result = vip_fluoro_init_mode(&fluoroPrms);
    printAsynMessage("vip_fluoro_init_mode", functionName, result);
    
    result = vip_set_user_sync(0, TRUE);
    printAsynMessage("vip_set_user_sync()", functionName, result);
    */

    // Set the sequence parameter info
    SSeqPrms seqPrms;
    memset(&seqPrms, 0, sizeof(SSeqPrms));
    seqPrms.StructSize = sizeof(SSeqPrms);

    seqPrms.NumBuffers = numBuf;
    // StopAfterN = zero is interpreted as acquire continuous (writing to buffers in circular fashion)
    // this is desired, as EPICS will orchestrate when to stop acquisition
    seqPrms.StopAfterN = numFrm;
    //seqPrms.SumSize = 2; frame summing if it works

    result = vip_fluoro_set_prms(HCP_FLU_SEQ_PRMS, &seqPrms);
    printAsynMessage("vip_fluoro_set_prms", functionName, result);
    if (result != VIP_NO_ERR)
        return asynError;

    // we may not get all we want so ...
    numBuf = seqPrms.NumBuffers;
    if (numFrm > numBuf)
        numFrm = numBuf;

    // before starting the grabber we want to make sure that
    // we are calibrated and have correction files to do what's requested
    // set up the SCorrections structure we will need below
    SCorrections corr;
    memset(&corr, 0, sizeof(SCorrections));
    corr.StructSize = sizeof(SCorrections);
    result = vip_get_correction_settings(&corr);

    // Start the grabber
    SAcqPrms acqPrms;
    memset(&acqPrms, 0, sizeof(SAcqPrms));
    acqPrms.StructSize = sizeof(SAcqPrms);
    acqPrms.MarkPixels = 0; //don't mark pixels
    acqPrms.StartUp = HCP_REC_IDLE;
    //if you start idle it waits until the current idle frame finishes before starting to record
    //acqPrms.StartUp = HCP_REC_RECORDING;
    //if you start paused it grabs whatever frame is currently in the buffer
    //acqPrms.StartUp = HCP_REC_PAUSED;

    // Determine fluoro correction settings
    int _fluoroCorrections;
    getIntegerParam(FluoroCorrections, &_fluoroCorrections);
    // Setting CorrType=HCP_CORR_NONE effectively acts as an override to the settings in SCorrections.
    acqPrms.CorrType = (_fluoroCorrections) ? HCP_CORR_STD :
                       HCP_CORR_NONE;
    // Only if the user is responsible for making his
    // own corrections should the user set this.
    acqPrms.CorrFuncPtr = NULL; 

    result = vip_fluoro_grabber_start(&acqPrms);
    printAsynMessage("vip_fluoro_grabber_start()", functionName, result);

    if (result != VIP_NO_ERR)
    {
        int result2 = vip_reset_state();

        if (result2 != VIP_NO_ERR)
            printAsynMessage("vip_reset_state()", functionName, result);
        return asynError;
    }

    // A pointer to a SLivePrms structure is returned in the acqPrms
    GLiveParams = (SLivePrms*)acqPrms.LivePrmsPtr;

    // At this point the grabber should be running. Frames are being captured
    // direct into the ping-pong buffers (unless started PAUSED).
    // They are not yet being saved into the sequence buffers.

    GGrabbingIsActive = true;

    // Set up real-time image handling as needed --
    result = vip_fluoro_get_event_name(HCP_FG_FRM_TO_DISP,
                                       gFrameReadyName);

    // (If Just-In-Time corrections are implemented we can ask for the
    // event name that we will use to signal we are ready for a frame.)

    // Start the worker thread that will handle real-time sending of frames
    //  to EPICS
    int status = epicsThreadCreate("FluoroAcquisitionTask",
                                   epicsThreadPriorityMedium,
                                   epicsThreadGetStackSize(
                                       epicsThreadStackMedium),
                                   (EPICSTHREADFUNC)fluoroGrabTaskC,
                                   this) == NULL;
    if (status) {
        printf("ERROR: epicsThreadCreate failed for fluoro acquisition task\n");
        return asynError;
    }

    // Signal that x-rays are now required
    setShutter(1);

    // If we want we can set StopAfterN and startFromBufIdx at this point too..
    // result = vip_fluoro_record_start(numFrm, startFromBufIdx);
    result = vip_fluoro_record_start();
    printAsynMessage("vip_fluoro_record_start()", functionName, result);
    if (result != VIP_NO_ERR)
    {
        GGrabbingIsActive = false;
        vip_fluoro_grabber_stop();
        vip_reset_state();
        return asynError;
    }

    if (false)
    {
        // wait for 8 seconds, if after 8 seconds, no frames received then acknowledge error
        Sleep(8000);

        if ( GNumFrames == 0)
        {
            printf("\n\nError: Timed out waiting for frames.\n");
            GGrabbingIsActive = false;
            vip_reset_state();
            return asynError;
        }
    }

    // wait for the requested number of frames to complete
    int acquire = 1;
    while (acquire) {
        Sleep(500);
        // check if we finished the EPICS acquisition sequence
        //  if we have continueAcquisition will set ADAcquire to 0
        continueAcquisition(1);
        getIntegerParam(ADAcquire, &acquire);
    }

    GGrabbingIsActive = false;
    GNumFrames = 0;
    GLiveParams = NULL;

    result = vip_fluoro_record_stop();
    printAsynMessage("vip_fluoro_record_stop()", functionName, result);

    result = vip_fluoro_grabber_stop();
    printAsynMessage("vip_fluoro_grabber_stop()", functionName, result);

    setShutter(0);

    /* If storing all in buffer and saving at the end
    for(int i=0; i<numFrm; i++)
    {
        WORD* buf=NULL;
        result = vip_fluoro_get_buffer_ptr(&buf, i);
        sendFrameToEpics((USHORT *)imPtr);
    }
    */

    // Print end of acquisition report
    {
        SSeqStats seqStats;
        memset(&seqStats, 0, sizeof(SSeqStats));
        result = vip_fluoro_get_prms(HCP_FLU_STATS_PRMS, &seqStats);
        if (result != VIP_NO_ERR)
            printAsynMessage("vip_fluoro_get_prms()", functionName, result);
        else {
            printf("\nFluoro Sequence Stats: SmplFrms=%d; HookFrms=%d; "
                   "CaptFrms=%d; HookOverrun=%d; StartIdx=%d; "
                   "EndIdx=%d; CaptRate=%.3f",
                   seqStats.SmplFrms, seqStats.HookFrms, seqStats.CaptFrms,
                   seqStats.HookOverrun, seqStats.StartIdx,
                   seqStats.EndIdx, seqStats.CaptRate);
        }
    }
    vip_reset_state();

    updateVarianState(0);
    return asynSuccess;
}

/* realtime Fluoro image grabbing task, created when fluoro acquisitions are started */
void varian::fluoroGrabTask()
{
    int totalImagesNeeded = 10;

    int lastCnt = -1;
    const int timeoutval = 500;
    HANDLE  hFrameEvent = CreateEvent(NULL, FALSE, FALSE,
                                      gFrameReadyName);

    if ( hFrameEvent == NULL ) {
        printf("\nERROR: hFrameEvent is NULL\n");
        return ;
    }

    DWORD dwTickCountLastFrame = GetTickCount();

    while (GGrabbingIsActive && GLiveParams)
    {

        // needs this to give other background threads a chance to run
        Sleep(100);

        if (WaitForSingleObject(hFrameEvent, timeoutval) != WAIT_TIMEOUT)
        {

            if ( GLiveParams == NULL) {
                printf("\nERROR: GLiveParams is NULL\n");
                return ;
            }

            USHORT* imPtr = (USHORT*)GLiveParams->BufPtr;
            GNumFrames = GLiveParams->NumFrames;

            // If we have a new frame, send it to EPICS
            if (GNumFrames && GNumFrames != lastCnt)
            {
                lastCnt = GNumFrames;
                //printf("\n --Image available. Ptr=0x%08x; " "Number of frames captured=%2.0d in %4.0d ms.",
                //            imPtr, GNumFrames, (GetTickCount() - dwTickCountLastFrame));
                sendFrameToEpics(imPtr, 0 );
                dwTickCountLastFrame = GetTickCount();
            }
        }
    }

    if (hFrameEvent)
        CloseHandle(hFrameEvent);
    return;
}

/* Thread constantly waits for a signal to start one of the acquisition routines */
void varian::imageGrabTask()
{
    static char *functionName = "imageGrabTask";
    asynStatus status = asynSuccess;

    int result, numImages, acquire, _doubleMode;
    getIntegerParam(ADNumImages, &numImages);

    lock();

    while (1) {
        /* Is acquisition active? */
        getIntegerParam(ADAcquire, &acquire);

        result = HCP_NO_ERR;
        /* If we are not acquiring then wait for a semaphore that is given when acquisition is started */
        if (!acquire) {
            setIntegerParam(ADStatus, ADStatusIdle);
            callParamCallbacks();

            /* Wait for a signal that tells this thread that the transmission
             * has started and we can start asking for image buffers...     */
            asynPrint(pasynUserSelf, ASYN_TRACE_FLOW,
                      "%s::%s waiting for acquire to start\n",
                      driverName, functionName);
            /* Release the lock while we wait for an event that says acquire has started, then lock again */
            unlock();
            /* This event is signaled from the startCapture function*/
            epicsEventWait(startEventId_);


            lock();
            asynPrint(pasynUserSelf, ASYN_TRACE_FLOW, "%s::%s started!\n",
                      driverName, functionName);
            //These three PVs need updating at the start of an EPICS acquisition sequence (not varian)
            setIntegerParam(ADNumImagesCounter, 0);
            setIntegerParam(ADAcquire, 1);
            setIntegerParam(ADStatus, ADStatusWaiting);
        }
        
        this->masterCancelFlag = false;

        /* Make absoultely sure we are in a valid state, and ready for acquisitions
            This might be overkill. Takes less than one millisecond.  */
        {
            result = vip_reset_state();
            if (result != HCP_NO_ERR)
                printAsynMessage("vip_reset_state()", functionName, result);
            result = vip_enable_sw_handshaking(FALSE);
            if (result != HCP_NO_ERR)
                printAsynMessage("vip_enable_sw_handshaking(FALSE)", functionName,
                                 result);
        }

        getIntegerParam(DoubleMode, &_doubleMode);
        if (this->radMode) {
            //we are in radiographic mode

            // doublemode is only relevant for varianAcqType == 0 
            if (_doubleMode && (varianAcqType == 0)) {
                printAsynMessage("Performing double rad acq", functionName,
                                 VIP_NO_ERR);
                //keep x-ray on for the "offset frame" and sum it with the raw image
                performDoubleRadAcquisition();
            } else {
                printAsynMessage("Performing standard rad acq", functionName,
                                 VIP_NO_ERR);
                //standard Varian radiography sequence
                // uses hardware handshaking
                performRadAcquisition();
                //if we want to use the slower but more convenient 
                // software handshaking
                //performSwRadAcquisition();
            }
        } else {
            //we are in fluoro mode
            printAsynMessage("Performing fluoro acq", functionName,
                                 VIP_NO_ERR);
            result = performFluoroAcquisition();
        }
    }
}

int varian::toggleMode()
{
    static char *functionName = "toggleMode";

    //for some unknown reason this is needed to do multiple single prepulse
    // acquisitions in a row. 

    // test if prepulse, 2 and 3 are the prepulse modes 
    if( this->varianAcqType == 2 || this->varianAcqType == 3){
        // test if this is after the first acq in a sequence
        //if (this->acqInProg == TRUE )
        //{
            printf("Attempting to toggle mode during acq\n");

            // find a mode that is not in use (either 1 or 0)
            int newMode = ( this->modeNumber == 0 ) ? 1 : 0;
            int result =  vip_select_mode(newMode);  

            //back to current mode
            result = vip_select_mode(this->modeNumber); 
            printAsynMessage("vip_select_mode during acq", functionName,
                             result);
            return result;
        //}
    }
    return HCP_NO_ERR;
}

/* Prepare paxscan for Radiography and wait until first frame is recieved
   (sets HS_ACTIVE and waits until panel is Ready for Pulse
   then sets shutter, and waits for first frame) */
int varian::radFirstFrame(UQueryProgInfo CurrentStatus)
{
    static char *functionName = "radFirstFrame";
    int result = HCP_NO_ERR;

    ResetTickCounts();
    StartTickCount();

    result = toggleMode();

    // Set handswitch to "Active", this usually takes less than 1 millisecond
    result = doHandSwitch(HS_ACTIVE);
    printAsynMessage("doHandSwitch(HS_ACTIVE)", functionName,
                     result);
    if (result != HCP_NO_ERR) //fatal error
        return result;

    // Wait until panel tells us it is okay to x-ray it (when it is "ready for pulse") ~400 milliseconds
    result = QueryWaitOnReadyForPulse(CurrentStatus, TRUE, TIMEOUT_5_SEC,
                                      &masterCancelFlag, true);
    printAsynMessage("QueryWaitOnReadyForPulse", functionName, result);
    if (result != HCP_NO_ERR) //fatal error
       return result;

    // Send Open Shutter Signal
    setShutter(1);

    EndTickCount("Prep for exposure");
    PrintTickCounts();
    setDoubleParam(PrepareTime , this->gdwTick[0] * (0.001));
    ResetTickCounts();


    printAsynMessage("Waiting for first frame to finish", functionName,
                     VIP_NO_ERR);
    StartTickCount();
    // longest mode we have is 6 sec so 15 sec should be plenty
    // however, because this is first frame sometimes the x-ray needs extra time to reach set points
    result = QueryWaitOnNumFrames(CurrentStatus, 1, 1,  TIMEOUT_30_SEC,
                                  &masterCancelFlag);
    EndTickCount("First frame complete");
    PrintTickCounts();
    setDoubleParam(FirstFrameTime , this->gdwTick[0] * (0.001));
    ResetTickCounts();
    StartTickCount();
    return result;
}

/* Wait for second radiography frame */
void varian::radSecondFrame(UQueryProgInfo CurrentStatus)
{
    static char *functionName = "radSecondFrame";
    // Varian Polling sequence to wait for the second of two frames
    // This always takes exposure length time + an additional 3 seconds (WHY???)
   
    printAsynMessage("Waiting for second frame to finish", functionName,
                     VIP_NO_ERR);
    // longest mode we have is 6 sec so 15 sec is plenty
    int result = QueryWaitOnNumFrames(CurrentStatus, 2, 1,
                                      TIMEOUT_15_SEC, &masterCancelFlag);
    EndTickCount("Second Frame Complete");
    PrintTickCounts();
    setDoubleParam(SecondFrameTime , this->gdwTick[0] * (0.001));
    ResetTickCounts();
    StartTickCount();
    return;
}

/* Tasks to be completed when finished with a single varian
   Radiography sequence, returns when panel is in Standby mode */
void varian::endRadiography(UQueryProgInfo CurrentStatus)
{
    static char *functionName = "endRadiography";

    int result = doHandSwitch(HS_STANDBY);
    printAsynMessage("doHandSwitch(HS_STANDBY)", functionName, result);

    result = QueryWaitOnStandby(CurrentStatus, TIMEOUT_5_SEC,
                                &masterCancelFlag);
    printAsynMessage("QueryWaitOnStandby", functionName, result);

    EndTickCount("End Radiography");
    PrintTickCounts();
    setDoubleParam(EndRadiographyTime, this->gdwTick[0] * (0.001));
    ResetTickCounts();
    return;
}

/*  Check to see if we have acquired enough frames to satisfy the
    user-set epics capture parameters, if so set Acquire to 0.
    Accepts "number of varian frames per acquisition" as an argument.
    For example, if we are saving exposed and offset, this would be 2,
    but if we are only saving corrected it would be 1 */
void varian::continueAcquisition(int varFramesPerAcq)
{
    static char *functionName = "continueAcquisition";
    int numImagesCounter, numImages, imageMode, numComplete;

    getIntegerParam(ADNumImagesCounter, &numImagesCounter);
    getIntegerParam(ADNumImages, &numImages);
    getIntegerParam(ADImageMode, &imageMode);

    numComplete =  numImagesCounter / varFramesPerAcq;

    //when this is called a frame has already been taken
    this->acqInProg = TRUE;
    
    printf("\nvarFramesPerAcq %i numImages %i numImagesCounter %i numComplete %i\n", varFramesPerAcq, numImages, numImagesCounter, numComplete);
    
    if ((imageMode == ADImageSingle) || ((imageMode == ADImageMultiple) &&
                                         (numComplete >= numImages))) {
        //Epics acquisition sequence is finished, this is how scan knows
        // the acquisition is complete 
        setIntegerParam(ADAcquire, 0);
        callParamCallbacks();
        printAsynMessage("Single EPICS Acquisition sequence complete", functionName, HCP_NO_ERR);
        this->acqInProg = FALSE;
    }
}

/* Leave x-ray on for both rad exposures, ignore all corrections and sum them together
   This effectively gives us one "uncorrected" double exposure.
   However, x-ray remains on for readout of both frames */
asynStatus varian::performDoubleRadAcquisition()
{
    static char *functionName = "performDoubleRadAcquisition";
    asynStatus status = asynSuccess;
    int result;
    UQueryProgInfo CurrentStatus;
    USHORT *pRawFrame  = new USHORT[sensorSize];
    USHORT *pDarkFrame = new USHORT[sensorSize];

    //this->varianState=1;
    updateVarianState(1);

    // First leg of Rad sequence, returns when first frame is complete
    result = radFirstFrame(CurrentStatus);
    printAsynMessage("First radiography frame", functionName, result);
    if (result != HCP_NO_ERR) {
        setShutter(0); // since there was an error we should shut off x-ray ASAP
        setIntegerParam(ADAcquire, 0); //end acquistiion
    }

    if (result == HCP_NO_ERR) {
        // Grab the first frame, but don't do anything with it yet
        result = vip_get_image(this->modeNumber, VIP_CURRENT_IMG_RAW ,
                               this->sensorWidth, this->sensorHeight, pRawFrame);
        printAsynMessage("vip_get_image()", functionName, result);

        // Second Rad frame, returns when second frame is complete
        radSecondFrame(CurrentStatus);
        setShutter(0);

        // Grab the second frame
        result = vip_get_image(this->modeNumber,
                               this->modeNumber == 0 ? VIP_OFFSET_IMG_0 : VIP_OFFSET_IMG_1 ,
                               this->sensorWidth, this->sensorHeight, pDarkFrame);
        printAsynMessage("vip_get_image()", functionName, result);
    }

    if (result == HCP_NO_ERR) {
        // Sum the first and second frames, and store in the pRawFrame buffer
        for ( int c = 0 ; c < sensorSize ; c++ )
            pRawFrame[c] += pDarkFrame[c];
        // "Send" summed frame to EPICS
        sendFrameToEpics(pRawFrame, 0);

        // Check to see if we have acquired enough frames to end Acquisition
        //  in this mode it is always 1 frame per "varian acquisition"
        continueAcquisition(1);
    }

    readParameters(); //update parameters

    // End radiography sequence, this returns the panel to standby mode
    // this should be called whether an acquisition is canceled or successful
    endRadiography(CurrentStatus);

    updateVarianState(0);

    callParamCallbacks();
    return asynSuccess;
}

/* Take a standard Varian rad acquisition sequence */
asynStatus varian::performRadAcquisition()
{
    static char *functionName = "performRadAcquisition";
    UQueryProgInfo CurrentStatus;

    updateVarianState(1);

    // First check to see which files user wants to save
    int _saveRaw, _saveOffset, _saveCorrected, _savePreview, _acquisitionType;
    getIntegerParam(SaveRaw, &_saveRaw);
    getIntegerParam(SaveOffset, &_saveOffset);
    getIntegerParam(SaveCorrected, &_saveCorrected);
    getIntegerParam(SavePreview, &_savePreview);
    getIntegerParam(VarianAcquisitionType, &_acquisitionType);
    printf("performRadAcquisition, acquisitiontype : %d \n", _acquisitionType);

    // First leg of Rad sequence, returns when first frame is complete
    int result = radFirstFrame(CurrentStatus);
    setShutter(0);
    if (result != HCP_NO_ERR) {
        printAsynMessage("Error during/before first radiography frame",
                         functionName, result);
        setIntegerParam(ADAcquire, 0); //end acquistiion
    }
    if (result == HCP_NO_ERR && _saveRaw)
        extractFrame(VIP_CURRENT_IMG_RAW);

    // Only proceed if first frame was successful and we are in post-offset radiography
    if (result == HCP_NO_ERR && _acquisitionType == 0 ) {

        // Second leg of Rad sequence, returns when second frame is complete
        radSecondFrame(CurrentStatus);
        if (_saveOffset)
            extractFrame(VIP_OFFSET_IMG_1);

        if (_saveCorrected)
            extractFrame(VIP_CURRENT_IMG_1);

        readParameters(); //update parameters
        continueAcquisition(_saveRaw + _saveCorrected + _saveOffset);

    } else if (result == HCP_NO_ERR && _acquisitionType == 2) {
        // extract the "preview" frame, this is only useful in 500ms Prepulse mode
        // which we intend for use in aligning samples (since prepulse doesn't seem
        // to do a post-offset frame) or for single 500ms raw frames
        //QueryWaitOnComplete(CurrentStatus, TIMEOUT_10_SEC, &masterCancelFlag);
        if (_savePreview)
            extractFrame(VIP_CURRENT_IMG_1);

        readParameters();
        continueAcquisition(_saveRaw + _savePreview);

        //needed?
        //result = vip_reset_state();
    }

    // End radiography sequence, this returns the panel to standby mode
    // this should be called whether an acquisition is canceled or successful
    // so we return to standby cleanly
    endRadiography(CurrentStatus);

    updateVarianState(0);
    callParamCallbacks();

    if (result != HCP_NO_ERR)
        return asynError;

    return asynSuccess;
}

/* This seems to take an average 1 second longer per acquisition sequence when compared to using
   hardware syncing. */
void  varian::performSwRadAcquisition()
{
    UQueryProgInfo CurrentStatus;
    bool CancelFlag = FALSE;
    int result = HCP_NO_ERR;
    char szFileName[MAX_STR];

    updateVarianState(1);

    // First check to see which files user wants to save
    int _saveRaw, _saveOffset, _saveCorrected, _savePreview;
    getIntegerParam(SaveRaw, &_saveRaw);
    getIntegerParam(SaveOffset, &_saveOffset);
    getIntegerParam(SaveCorrected, &_saveCorrected);
    getIntegerParam(SavePreview, &_savePreview);

    // initialize tickcounts
    ResetTickCounts();
    StartTickCount();
    printf( "\nAcquiring sw image for mode #%d %s\n\n", this->modeNumber, gModeInfo.ModeDescription);

    //if (result == HCP_NO_ERR)
    {
        printf("\nChecking mode...");
        int ModeType=-1, AcquisitionNumber=-1;
        result = vip_get_mode_acq_type(this->modeNumber, &ModeType, &AcquisitionNumber);
        if(result == HCP_NO_ERR)
        {
            printf("Mode settings; ModeType=%d; AcquisitionNumber=%d", ModeType, AcquisitionNumber);

            if(ModeType != VIP_VALID_XRAYS_N_FRAMES || AcquisitionNumber <= 0)
            {
                printf("Problem with mode settings; ModeType=%d; AcquisitionNumber=%d", ModeType, AcquisitionNumber);
                result = HCP_OTHER_ERR;
            }
        }

    }

    // Enable software handshaking
    if (result == HCP_NO_ERR)
    {
        printf("\nEnabling software handshaking...");
        result = vip_enable_sw_handshaking(TRUE);
    }

    // Set PREPARE = TRUE
    if (result == HCP_NO_ERR)
    {
        printf("\nSetting PREPARE=TRUE...");
        result = vip_sw_handshaking(VIP_SW_PREPARE, TRUE);
    }

    // Wait for ReadyForPulse to go TRUE
    if (result == HCP_NO_ERR)
    {
        result = QueryWaitOnReadyForPulse(CurrentStatus, 1, 60000, NULL, true);
        EndTickCount("QueryWaitOnReadyForPulse tickCount");
        PrintTickCounts();
        setDoubleParam(PrepareTime , this->gdwTick[0] * (0.001));
        ResetTickCounts();
        StartTickCount();
    }
    setShutter(1);

    // Set XRAY_VALID = TRUE
    if (result == HCP_NO_ERR)
    {
        printf("Setting XRAY_VALID=TRUE...\n");
        result = vip_sw_handshaking(VIP_SW_VALID_XRAYS, TRUE);
    }

    QueryWaitOnNumFrames(CurrentStatus, 1, 1, TIMEOUT_15_SEC , NULL);
    EndTickCount("First frame complete");
    PrintTickCounts();
    setDoubleParam(FirstFrameTime , this->gdwTick[0] * (0.001));
    ResetTickCounts();
    StartTickCount();

    setShutter(0);
    // save the captured image to file
    if (result == HCP_NO_ERR && _saveRaw)
        extractFrame(VIP_CURRENT_IMG_RAW);

    if (result == HCP_NO_ERR)
    {
        printf("\n\nREADY FOR XRAYS!!");

        printf("\nWaiting for Complete == TRUE...");

        StartTickCount();
        result = QueryWaitOnComplete(CurrentStatus, 0, &CancelFlag);
        EndTickCount("QueryWaitOnComplete");
        PrintTickCounts();
        setDoubleParam(SecondFrameTime , this->gdwTick[0] * (0.001));
        ResetTickCounts();
        StartTickCount();
        if (CancelFlag) // ensure that we exit operation if user requested a cancel
            result = -1;
    }

    // be sure the grabber stops Set PREPARE = FALSE
    if (result == HCP_NO_ERR)
    {
        printf("\nSetting PREPARE=FALSE...\n");
        result = vip_sw_handshaking(VIP_SW_PREPARE, FALSE);
        //if (result)
        //    PrintError("vip_sw_handshaking(VIP_SW_PREPARE, FALSE)", result, __LINE__, __FILE__);
    }

    // Only proceed if first frame was successful and we are not in prepulse mode
    if (result == HCP_NO_ERR && this->modeNumber != 0) {
        if (_saveOffset)
            extractFrame(VIP_OFFSET_IMG_1);

        if (_saveCorrected)
            extractFrame(VIP_CURRENT_IMG_1);
    }

    continueAcquisition(_saveRaw + _saveCorrected + _saveOffset);
    EndTickCount("End Radiography");
    PrintTickCounts();
    setDoubleParam(EndRadiographyTime, this->gdwTick[0] * (0.001));
    ResetTickCounts();

    updateVarianState(0);

    return;
}

/* Called when Acquire is caput to 1 */
asynStatus varian::startCapture()
{
    static const char *functionName = "startCapture";
    UQueryProgInfo CurrentStatus;

    /* Start the camera transmission... */
    setIntegerParam(ADNumImagesCounter, 0);
    QueryProgress(CurrentStatus);
    asynPrint(pasynUserSelf, ASYN_TRACE_WARNING,
              "%s::%s calling CameraBase::StartCapture\n",
              driverName, functionName);
    //  signal image grab task to begin grabbing images from the panel,
    epicsEventSignal(startEventId_);
    return asynSuccess;
}

/* Here is where we would implement cancel flag */
/* Called when Acquire is caput to 0 */
asynStatus varian::stopCapture()
{
    static const char *functionName = "stopCapture";
    setIntegerParam(ADAcquire, 0);
    DoCancel();
    return asynSuccess;
}

/* Check which mode and call appropriate gain cal sequence */
void varian::performGainCal()
{
    setIntegerParam(ADStatus, ADStatusCorrect);
    lock();

   if (this->varianAcqType == 0){
        printf("perform radiography gain cal\n");
        performGainCalRad();
    } else if (this->varianAcqType == 2 || 3){
        printf("perform prepulse gain cal\n");
        performGainCalPrePulse();
    } else if (this->varianAcqType == 4){
        printf("perform fluoro gain cal\n");
        performGainCalFluoro();
    }

    // just to be sure we return to a normal state
    vip_reset_state();
    unlock();

    setIntegerParam(GainCal, 0);
    setIntegerParam(ADStatus, ADStatusIdle);
    // update calibration info PV's
    changeMode(this->modeNumber);
}

/*  Varian Rad Gain Calibration routine
    results in the creation of new Gain/Defect/Offset images
    stored in the receptor config folder for a given mode
    Unlike fluoro gain cal, this process uses hardware sync signals
    (similar to a standard rad acquisition) */
void varian::performGainCalRad()
{
    static char *functionName = "performGainCalRad";
    UQueryProgInfo CurrentStatus;
    int NumberGainCalImages;  // Varian recommends 32
    int NumberOffsetCal;      // Varian recommends 10
    asynStatus status = asynSuccess;

    updateVarianState(3);

    masterCancelFlag = false;
    getIntegerParam(GainCalNumOffsets, &NumberOffsetCal);
    getIntegerParam(GainCalNumFlats, &NumberGainCalImages);

    printf("\n##########################################################\n");
    printf("\nPerforming gain calibration for mode %d %s",
           this->modeNumber , gModeInfo.ModeDescription);
    printf("\nwith %d offsets and %d gain images ", NumberOffsetCal,
           NumberGainCalImages);
    printf("\nIf succesful new gain_img.viv/ofst_img.viv/defect_map.bin files located in %s",
           this->gszIniPath);
    printf("\nin the respective mode number folder.\n");

    printf("\nThere must be no objects in between the x-ray and detector. ");
    printf("\nThe user must set appropriate kVp/uA for gain calibration. ");
    printf("\nThe flat field image should be within a range that is large enough to be ");
    printf("\nhigher than the background noise created by the X-ray source/readout of the ");
    printf("\nReceptor, but lower than the saturation point. Median count should be ");
    printf("\n~1,600-3,000 in flat field images. For Rad- Varian recommends 32 frames for gain calibration, 8 frames for offset.");
    printf("\n##########################################################\n");

    // reset state just in case we are not in a valid state
    int result = vip_reset_state();
    printAsynMessage("vip_reset_state()", functionName, result);

    // retrieve the number of images required for a gain calibration
    result = vip_set_num_cal_frames(this->modeNumber, NumberOffsetCal);
    printAsynMessage("vip_set_num_cal_frames()", functionName, result);

    if (result == HCP_NO_ERR)
    {
        result = vip_gain_cal_prepare(this->modeNumber, FALSE);
        printAsynMessage("vip_gain_cal_prepare()", functionName, result);
    }

    if (result == HCP_NO_ERR)
    {
        result = vip_sw_handshaking(VIP_SW_PREPARE, TRUE);
        printAsynMessage("vip_sw_handshaking()", functionName, result);
    }

    if (result == HCP_NO_ERR)
    {
        printf("Acquiring initial dark fields - DO NOT EXPOSE TO X-RAYS\n");
        result = QueryWaitOnNumFrames(CurrentStatus, NumberOffsetCal, 1,
                                      TIMEOUT_15_SEC * NumberOffsetCal, &masterCancelFlag);
        if (result == HCP_NO_ERR)
            printf("Initial offset calibration complete\n\n");
        else
            printf("QueryWaitOnNumFrames()");
    }

    result = vip_enable_sw_handshaking(FALSE);
    printAsynMessage("vip_enable_sw_handshaking(FALSE)", functionName,
                     result);

    if (result == HCP_NO_ERR)
    {
        int NumberFrames = 0;

            result = doHandSwitch(HS_ACTIVE);

        // Capture the required number of flat-field images
        // We could have x-ray on for this entire loop, and then use setShutter just to handle the exp-req signals
        // However, this would require extra code to handle this special case
        // Instead, we have it behave exactly as we do in performRadAcquisition so our sync code can treat this just
        // like a normal rad acquisition. This should also result in a more consistent gain cal image.
        for (NumberFrames = 0; NumberFrames < NumberGainCalImages; )
        {
            printf("Flat field exposure # %d\n", NumberFrames);
               
            // Set handswitch to "Active", this usually takes less than 1 millisecond
            printAsynMessage("doHandSwitch(HS_ACTIVE)", functionName,
                     result);

           result = QueryWaitOnReadyForPulse(CurrentStatus, TRUE, TIMEOUT_5_SEC,
                                              NULL, true);

            // check for error or cancellation request
            if (result != HCP_NO_ERR)
            {
                printAsynMessage("QueryWaitOnReadyForPulse", functionName, result);
                return;
            }

            if (result == HCP_NO_ERR)
            {
                //Open EPICS Shutter
                setShutter(1);

                if (result == HCP_NO_ERR)
                {
                    // 2 frames per acquisition
                    result = QueryWaitOnNumFrames(CurrentStatus,
                                                  2,
                                                  2,
                                                  TIMEOUT_15_SEC,
                                                  NULL);

                    // check for error or cancellation request
                    if ((result != HCP_NO_ERR))
                    {
                        printAsynMessage("QueryWaitOnNumFrames", functionName, result);
                        this->acqInProg = FALSE;
                        //return;
                    }
                }
                setShutter(0);
                // make task switch to terminate calibration thread
                Sleep(2000);
                this->acqInProg = TRUE;
                // update the number of frames
                NumberFrames += CurrentStatus.qpi.NumFrames;
                printf("numframes %i finished out of numgaincalimages %i\n",
                       NumberFrames, NumberGainCalImages);
               
                printAsynMessage("doHandSwitch(HS_STANDBY)", functionName, result);
            }
        }
    }
                result = doHandSwitch(HS_STANDBY);

    this->acqInProg = FALSE;

    if (result == HCP_NO_ERR)
    {
        // clear the prepare signal
        // this will cause virtual CP to begin processing the captured images
        // and calculate the corrections data
        result = vip_sw_handshaking(VIP_SW_PREPARE, FALSE);
        printAsynMessage("vip_sw_handshaking(VIP_SW_PREPARE, FALSE)",
                         functionName, result);
    }

    if (result == HCP_NO_ERR)
    {
        // wait for the virtual CP to finish processing gain cal image data
        result = QueryWaitOnComplete(CurrentStatus, TIMEOUT_10_SEC,
                                     &masterCancelFlag);
        printAsynMessage("QueryWaitOnComplete()", functionName, result);
    }

    // if an error occured or a cancel was requested reset the state of the virtual CP
    if (result != HCP_NO_ERR)
        return;

    updateVarianState(0);
    printf("Gain calibration for mode %d %s completed. Be sure to verify resulting gain image\n",
           this->modeNumber , gModeInfo.ModeDescription);
    return;
}

/* for some reason prepulse mode (single frame radiography) requires this sequence 
    which involves attempting to switch modes and then getting an error message. */
//todo: merge with performGainCalRad function
void varian::performGainCalPrePulse()
{
    static char *functionName = "performSwGainCal";
    UQueryProgInfo CurrentStatus;
    int NumberGainCalImages;  // Varian recommends 32
    int NumberOffsetCal;      // Varian recommends 10
    asynStatus status = asynSuccess;

    updateVarianState(3);

    masterCancelFlag = false;
    getIntegerParam(GainCalNumOffsets, &NumberOffsetCal);
    getIntegerParam(GainCalNumFlats, &NumberGainCalImages);

    printf("\n##########################################################\n");
    printf("\nPerforming gain calibration for mode %d %s",
           this->modeNumber , gModeInfo.ModeDescription);
    printf("\nwith %d offsets and %d gain images ", NumberOffsetCal,
           NumberGainCalImages);
    printf("\nIf succesful new gain_img.viv/ofst_img.viv/defect_map.bin files located in %s",
           this->gszIniPath);
    printf("\nin the respective mode number folder.\n");
    printf("\nThere must be no objects in between the x-ray and detector. ");
    printf("\nThe user must set appropriate kVp/uA for gain calibration. ");
    printf("\nThe flat field image should be within a range that is large enough to be ");
    printf("\nhigher than the background noise created by the X-ray source/readout of the ");
    printf("\nReceptor, but lower than the saturation point. Median count should be ");
    printf("\n~1,600-3,000 in flat field images. For Rad- Varian recommends 32 frames for gain calibration, 8 frames for offset.");
    printf("\n##########################################################\n");

    // reset state just in case we are not in a valid state
    int result = vip_reset_state();
    printAsynMessage("vip_reset_state()", functionName, result);

    // retrieve the number of images required for a gain calibration
    result = vip_set_num_cal_frames(this->modeNumber, NumberOffsetCal);
    printAsynMessage("vip_set_num_cal_frames()", functionName, result);

    if (result == HCP_NO_ERR)
    {
        result = vip_gain_cal_prepare(this->modeNumber, FALSE);
        printAsynMessage("vip_gain_cal_prepare()", functionName, result);
    }

    if (result == HCP_NO_ERR)
    {
        result = vip_sw_handshaking(VIP_SW_PREPARE, TRUE);
        printAsynMessage("vip_sw_handshaking()", functionName, result);
    }

    if (result == HCP_NO_ERR)
    {
        printf("Acquiring initial dark fields - DO NOT EXPOSE TO X-RAYS\n");
        result = QueryWaitOnNumFrames(CurrentStatus, NumberOffsetCal, 1,
                                      TIMEOUT_15_SEC * NumberOffsetCal, &masterCancelFlag);
        if (result == HCP_NO_ERR)
            printf("Initial offset calibration complete\n\n");
        else
            printf("QueryWaitOnNumFrames()");
    }

    result = vip_enable_sw_handshaking(TRUE);
    printAsynMessage("vip_enable_sw_handshaking(FALSE)", functionName,
                     result);

    if (result == HCP_NO_ERR)
    {

            UQueryProgInfo CurrentStatus2;
            // Set handswitch to "Active", this usually takes less than 1 millisecond
            result = doHandSwitch(HS_ACTIVE);
            printAsynMessage("doHandSwitch(HS_ACTIVE)", functionName,
                     result);      


           result = QueryWaitOnReadyForPulse(CurrentStatus2, TRUE, TIMEOUT_UNLIMITED,
                                              NULL, true);

            // check for error or cancellation request
            if (result != HCP_NO_ERR)
            {
                printAsynMessage("QueryWaitOnReadyForPulse", functionName, result);
                vip_reset_state();
                return;
            }

        int NumberFrames = 0;
        // Capture the required number of flat-field images
        // We could have x-ray on for this entire loop, and then use setShutter just to handle the exp-req signals
        // However, this would require extra code to handle this special case
        // Instead, we have it behave exactly as we do in performRadAcquisition so our sync code can treat this just
        // like a normal rad acquisition. This should also result in a more consistent gain cal image.
        for (NumberFrames = 0; NumberFrames < NumberGainCalImages; )
        {
            printf("Flat field exposure # %d\n", NumberFrames);
            
            if (this->acqInProg == TRUE)
                toggleMode();
            if (result == HCP_NO_ERR)
            {
                //Open EPICS Shutter
                setShutter(1);

                if (result == HCP_NO_ERR)
                {
                    // 1 frame per acquisition
                    result = QueryWaitOnNumFrames(CurrentStatus2,
                                                  1,
                                                  1,
                                                  TIMEOUT_30_SEC,
                                                  NULL);

                    // check for error or cancellation request
                    if ((result != HCP_NO_ERR))
                    {
                        printAsynMessage("QueryWaitOnNumFrames", functionName, result);
                        this->acqInProg = FALSE;
                        break;
                    }
                }
                setShutter(0);
                // make task switch to terminate calibration thread
                Sleep(2000);
                this->acqInProg = TRUE;
                // update the number of frames
                NumberFrames += CurrentStatus2.qpi.NumFrames;
                printf("numframes %i finished out of numgaincalimages %i\n",
                       NumberFrames, NumberGainCalImages);
            }
        }
    }

    result = doHandSwitch(HS_STANDBY);
    printAsynMessage("doHandSwitch(HS_STANDBY)", functionName, result);
    
    this->acqInProg = FALSE;

    if (result == HCP_NO_ERR)
    {
        // clear the prepare signal
        // this will cause virtual CP to begin processing the captured images
        // and calculate the corrections data
        result = vip_sw_handshaking(VIP_SW_PREPARE, FALSE);
        printAsynMessage("vip_sw_handshaking(VIP_SW_PREPARE, FALSE)",
                         functionName, result);
    }

    if (result == HCP_NO_ERR)
    {
        // wait for the virtual CP to finish processing gain cal image data
        result = QueryWaitOnComplete(CurrentStatus, TIMEOUT_10_SEC,
                                     &masterCancelFlag);
        printAsynMessage("QueryWaitOnComplete()", functionName, result);
    }

    // if an error occured or a cancel was requested reset the state of the virtual CP
    if (result != HCP_NO_ERR)
        return;

    updateVarianState(0);
    printf("Gain calibration for mode %d %s completed. Be sure to verify resulting gain image\n",
           this->modeNumber , gModeInfo.ModeDescription);
    return;
} 

/* Perform a Varian gain calibration sequence for fluoro mode.
   Unlike the gain calibration sequence for Rad mode, we are forced
   to use software syncing for this process. Therefore the panel ignores
   exp req signals, and doesn't send exp ok signals.
   This means our python sync application has to treat setShutter differently in this
   process. In this case, setShutter(1) just informs python to turn x-ray on, and setShutter(0)
   means turn x-ray off. There is no communication using the sync cables. */
void varian::performGainCalFluoro()
{
    static char *functionName = "performGainCalFluoro";
    int numCalFrmSet = 0;
    UQueryProgInfo CurrentStatus;

    updateVarianState(4);
    int result = vip_get_num_cal_frames(this->modeNumber, &numCalFrmSet);
    printAsynMessage("vip_get_num_cal_frames", functionName, result);
    if (result != VIP_NO_ERR)
        return;

    printf("\n\nPerformGainCalFluoro with %i frames\n", numCalFrmSet);

    // tell the system to prepare for a gain calibration
    result = vip_gain_cal_prepare(this->modeNumber, FALSE);
    printAsynMessage("vip_gain_cal_prepare", functionName, result);
    if (result != VIP_NO_ERR)
        return;

    // send prepare = true
    result = vip_sw_handshaking(VIP_SW_PREPARE, TRUE);
    printAsynMessage("vip_sw_handshaking(VIP_SW_PREPARE)", functionName,
                     result);

    if (result != VIP_NO_ERR)
        return;

    SQueryProgInfo qpi;
    memset(&qpi, 0, sizeof(SQueryProgInfo));
    qpi.StructSize = sizeof(SQueryProgInfo);
    UQueryProgInfo* uqpi = (UQueryProgInfo*)&qpi;

    // changed to wait 1 frame, then commented out rfp
    printf("\n\nWaiting for RFP\n");
    result = QueryWaitOnReadyForPulse(CurrentStatus, TRUE, TIMEOUT_5_SEC,
                                      NULL, true);
    printAsynMessage("QueryWaitOnReadyForPulse", functionName, result);
    if (result != VIP_NO_ERR)
        return;
    printf("rfp complete\n");

    printf("\nSetShutter(1)\n");
    setShutter(1);
    //Sleep(5000); //temporary

    // send xrays = true - this signals the START of the FLAT-FIELD ACQUISITION
    result = vip_sw_handshaking(VIP_SW_VALID_XRAYS, TRUE);
    printAsynMessage("vip_sw_handshaking(VIP_SW_VALID_XRAYS, TRUE)",
                     functionName, result);
    if (result != VIP_NO_ERR)
        return;

    int NumberFrames = 0;
    // Capture the required number of flat-field images
    for (NumberFrames = 0; NumberFrames < numCalFrmSet; )
    {
        printf("Gain calibration:  Flat field %d\n", NumberFrames);

        // 2 frames per acquisition for fluoro
        result = QueryWaitOnNumFrames(CurrentStatus,
                                      1,
                                      1,
                                      TIMEOUT_5_SEC,
                                      NULL);

        printAsynMessage("QueryWaitOnNumFrames", functionName, result);
        // check for error or cancellation request
        if ((result != HCP_NO_ERR))
            return;

        // make task switch to terminate calibration thread
        Sleep(2000);

        // update the number of pulses
        NumberFrames += CurrentStatus.qpi.NumFrames;
        printf("numframes %i finished out of %i\n", NumberFrames,
               numCalFrmSet);

    }
    setShutter(0);

    Sleep(1000); //needed?
    // send xrays = false - this signals the START of the DARK-FIELD ACQUISITION
    result = vip_sw_handshaking(VIP_SW_VALID_XRAYS, FALSE);
    printAsynMessage("vip_sw_handshaking(VIP_SW_VALID_XRAYS, FALSE)",
                     functionName, result);
    if (result != VIP_NO_ERR)
        return;

    Sleep(1000); //needed?

    // wait for the calibration to complete
    qpi.Complete = FALSE;
    while (!qpi.Complete && result == VIP_NO_ERR)
    {
        result = vip_query_prog_info(HCP_U_QPI, uqpi);
        if (qpi.NumFrames < numCalFrmSet)
            printf("\nDark-field frames accumulated = %d", qpi.NumFrames);
        Sleep(100);
    }

    if (result != VIP_NO_ERR) {
        printAsynMessage("vip_query_prog_info", functionName, result);
        return;
    }

    updateVarianState(0);
    printf("\nFluoro Gain calibration completed successfully");

    return;
}


/** Called when asyn clients call pasynInt32->write().
  * This function performs actions for some parameters, including ADAcquire, ADBinX, etc.
  * For all parameters it sets the value in the parameter library and calls any registered callbacks..
  * \param[in] pasynUser pasynUser structure that encodes the reason and address.
  * \param[in] value Value to write. */
asynStatus varian::writeInt32(asynUser *pasynUser, epicsInt32 value)
{
    static const char *functionName = "writeInt32";
    int function = pasynUser->reason;
    asynStatus status = asynSuccess;

    /* Set the parameter and readback in the parameter library.  This may be overwritten when we read back the
     * status at the end, but that's OK */
    status = setIntegerParam(function, value);

    if (function == ADAcquire) {
        if (value) {
            setIntegerParam(ADStatus, ADStatusWaiting);
            status = startCapture();
        } else {
            setIntegerParam(ADStatus, ADStatusIdle);
            status = stopCapture();
        }
    } else if (function == VarianConfig)
        { if (value != currConfigNum){
            changeConfig(value);}}
    else if (function == VarianMode || function == VarianModeFluoro)
        changeMode(value);
    else if (function == OffsetCorrection || function == GainCorrection ||
             function == DefectCorrection)
        changeCorrections();
    else if (function == DebugMode)
        changeDebug(value);
    else if (function == CheckPaxscanLink)
        checkLink();
    else if (function == GainCal)
        performGainCal();
    else if (function == AnalogOffsetCal)
        performRadAnalogOffsetCal();
    else if (function == ExtractOffsetFrame )
        extractFrame(VIP_OFFSET_IMAGE);
    else if (function == ExtractGainFrame )
        extractFrame(VIP_GAIN_IMAGE);
    else if (function == ExtractDefectFrame )
        extractFrame(VIP_BASE_DEFECT_IMAGE);
    else if (function == ExtractAuxDefectFrame )
        extractFrame(VIP_AUX_DEFECT_IMAGE);

    /* Read the camera parameters and do callbacks */
    status = readParameters();

    if (status)
        asynPrint(pasynUser, ASYN_TRACE_ERROR,
                  "%s:%s: error, status=%d function=%d, value=%d\n",
                  driverName, functionName, status, function, value);
    else
        asynPrint(pasynUser, ASYN_TRACEIO_DRIVER,
                  "%s:%s: function=%d, value=%d\n",
                  driverName, functionName, function, value);
    return ((asynStatus)status);
}


extern "C" int varianConfig(char *portName, /* Port name */
                            char *gszIniPathRad, char *gszIniPathFluoro, char *gszIniPathCustom1, 
                            char *gszIniPathCustom2, 
                            int bootIniPath, int checkLinkOnConnect,
                            int maxBuffers, size_t maxMemory, int priority, int stackSize)
{
    new varian(portName, gszIniPathRad, gszIniPathFluoro, gszIniPathCustom1, gszIniPathCustom2, bootIniPath,
               checkLinkOnConnect, maxBuffers,
               maxMemory, priority, stackSize);
    return (asynSuccess);
}

/** Constructor for Varian driver; most parameters are simply passed to ADDriver::ADDriver.
  * After calling the base class constructor this method creates a thread to collect the detector data,
  * and sets reasonable default values for the parameters defined in this class, asynNDArrayDriver and ADDriver.
  * \param[in] portName The name of the asyn port driver to be created.
  * \param[in] gszIniPathRad Varian receptor configuration folder location for Rad
  * \param[in] gszIniPathFluoro Varian receptor configuration folder location for Fluoro
  * \param[in] bootIniPath if 0 use gszIniPathRad if 1 use gszIniPathFluoro at boot
  * \param[in] checkLinkOnConnect whether to checklink everytime we reconnect to paxscan
  * \param[in] maxBuffers The maximum number of NDArray buffers that the NDArrayPool for this driver is
  *            allowed to allocate. Set this to -1 to allow an unlimited number of buffers.
  * \param[in] maxMemory The maximum amount of memory that the NDArrayPool for this driver is
  *            allowed to allocate. Set this to -1 to allow an unlimited amount of memory.
  * \param[in] priority The thread priority for the asyn port driver thread if ASYN_CANBLOCK is set in asynFlags.
  * \param[in] stackSize The stack size for the asyn port driver thread if ASYN_CANBLOCK is set in asynFlags.
  * \param[in] maxPvAPIFrames The number of frame buffers to use in the PvAPI library driver. Default=MAX_PVAPI_FRAMES=2.
  */
varian::varian(const char *portName, const char *gszIniPathRad,
               const char *gszIniPathFluoro, const char *gszIniPathCustom1, const char *gszIniPathCustom2, 
               int bootIniPath, int checkLinkOnConnect,
               int maxBuffers, size_t maxMemory, int priority, int stackSize)
    : ADDriver(portName, 1, NUM_VARIAN_PARAMS, maxBuffers, maxMemory,
               0, 0,               /* No interfaces beyond those set in ADDriver.cpp */
               ASYN_CANBLOCK,
               0,   /* ASYN_CANBLOCK=1, ASYN_MULTIDEVICE=0, autoConnect=1 */
               priority, stackSize),  pImage(NULL)

{
    int status = asynSuccess;
    static const char *functionName = "varian";

    // Prep the instance's mode information struct
    memset(&gModeInfo, 0, sizeof(gModeInfo));
    gModeInfo.StructSize = sizeof(SModeInfo);

    //settings
    createParam(VarianConfigString,     asynParamInt32,
                &VarianConfig);
    createParam(VarianModeString,       asynParamInt32,     &VarianMode);
    createParam(VarianAcquisitionTypeString,       asynParamInt32,     &VarianAcquisitionType);
    createParam(NumberOfModesString,    asynParamInt32,     &NumberOfModes);
    createParam(VarianModeFluoroString, asynParamInt32,
                &VarianModeFluoro);
    createParam(VarianModeDescriptionString, asynParamOctet,
                &VarianModeDescription);
    createParam(DebugModeString,        asynParamInt32,     &DebugMode);
    createParam(DoubleModeString,       asynParamInt32,     &DoubleMode);

    //corrections
    createParam(SetCorrectionsString,   asynParamInt32,
                &SetCorrections);
    createParam(OffsetCorrectionString, asynParamInt32,
                &OffsetCorrection);
    createParam(GainCorrectionString,   asynParamInt32,
                &GainCorrection);
    createParam(DefectCorrectionString, asynParamInt32,
                &DefectCorrection);
    createParam(FluoroCorrectionsString, asynParamInt32,
                &FluoroCorrections);

    //file saving
    createParam(SaveRawString,          asynParamInt32,   &SaveRaw);
    createParam(SaveOffsetString,       asynParamInt32,   &SaveOffset);
    createParam(SaveCorrectedString,    asynParamInt32,   &SaveCorrected);
    createParam(SavePreviewString,      asynParamInt32,   &SavePreview);

    createParam(PrepareTimeString,         asynParamFloat64,
                &PrepareTime);
    createParam(FirstFrameTimeString,      asynParamFloat64,
                &FirstFrameTime);
    createParam(SecondFrameTimeString,     asynParamFloat64,
                &SecondFrameTime);
    createParam(EndRadiographyTimeString,  asynParamFloat64,
                &EndRadiographyTime);

    //check link
    createParam(CheckPaxscanLinkString, asynParamInt32,
                &CheckPaxscanLink);

    //panel information
    createParam(BoardSerialNumberString,  asynParamOctet,
                &BoardSerialNumber);
    createParam(FirmwareVersionString,    asynParamInt32,
                &FirmwareVersion);
    createParam(ReceptorConfigPathString, asynParamOctet,
                &ReceptorConfigPath);
    createParam(MacAddressString,         asynParamOctet,   &MacAddress);
    createParam(IpAddressString,          asynParamOctet,   &IpAddress);

    //status info
    createParam(CompleteString,         asynParamInt32,   &Complete);
    createParam(ReadyForPulseString,    asynParamInt32,   &ReadyForPulse);
    createParam(NumFramesString,        asynParamInt32,   &NumFrames);
    createParam(NumPulsesString,        asynParamInt32,   &NumPulses);


    createParam(VarianStateString,      asynParamInt32, &VarianState);
    createParam(HandswitchStateString,  asynParamInt32, &HandswitchState);

    createParam(Temp1String,            asynParamFloat64, &Temp1);
    createParam(Temp2String,            asynParamFloat64, &Temp2);
    createParam(Temp3String,            asynParamFloat64, &Temp3);
    createParam(Temp4String,            asynParamFloat64, &Temp4);
    createParam(Temp5String,            asynParamFloat64, &Temp5);
    createParam(Temp6String,            asynParamFloat64, &Temp6);

    //stored mode/calibration info
    createParam(VarianFrameRateString,     asynParamFloat64,
                &VarianFrameRate);
    createParam(OffsetMedianString,        asynParamFloat64,
                &OffsetMedian);
    createParam(OffsetStdDevString,        asynParamFloat64,
                &OffsetStdDev);
    createParam(GainMedianString,          asynParamFloat64,
                &GainMedian);
    createParam(GainStdDevString,          asynParamFloat64,
                &GainStdDev);
    createParam(GainScalingString,         asynParamFloat64,
                &GainScaling);
    createParam(TimeLastCalibratedString,  asynParamOctet,
                &TimeLastCalibrated);

    //gain cal routine
    createParam(GainCalString,           asynParamInt32,   &GainCal);
    createParam(GainCalNumOffsetsString, asynParamInt32,
                &GainCalNumOffsets);
    createParam(GainCalNumFlatsString,   asynParamInt32,
                &GainCalNumFlats);

    //analog offset cal routine
    createParam(AnalogOffsetCalString,  asynParamInt32,
                &AnalogOffsetCal);

    //extract calibration frames
    createParam(ExtractOffsetFrameString,    asynParamInt32,
                &ExtractOffsetFrame);
    createParam(ExtractGainFrameString,      asynParamInt32,
                &ExtractGainFrame);
    createParam(ExtractDefectFrameString,    asynParamInt32,
                &ExtractDefectFrame);
    createParam(ExtractAuxDefectFrameString, asynParamInt32,
                &ExtractAuxDefectFrame);

    this->gszIniPathVarianRad = epicsStrDup(gszIniPathRad);
    this->gszIniPathVarianFluoro = epicsStrDup(gszIniPathFluoro);
    this->gszIniPathCustom1 = epicsStrDup(gszIniPathCustom1);
    this->gszIniPathCustom2 = epicsStrDup(gszIniPathCustom2);
    this->currConfigNum = bootIniPath;

    // determine which mode to boot into
    this->gszIniPath = bootIniPath ? this->gszIniPathVarianFluoro :
                       this->gszIniPathVarianRad;

    this->checkLinkOnConnect = checkLinkOnConnect ? true : false;

    status = connectCamera();
    this->unlock();
    if (status) {
        printf("%s:%s: cannot connect to camera, manually connect when available.\n",
               driverName, functionName);
        return;
    }

    startEventId_ = epicsEventCreate(epicsEventEmpty);
    status = (epicsThreadCreate("ImageAcquisitionTask",
                                epicsThreadPriorityMedium,
                                epicsThreadGetStackSize(
                                    epicsThreadStackMedium),
                                (EPICSTHREADFUNC)imageGrabTaskC,
                                this) == NULL);
    if (status) {
        printf("%s ERROR: epicsThreadCreate failed for image "
               "acquisition task\n", functionName);
        return;
    }

    /* Register the shutdown function for epicsAtExit */
    epicsAtExit(shutdown, (void*)this);
}

/* Code for iocsh registration */
static const iocshArg varianConfigArg0 = {"Port name", iocshArgString};
static const iocshArg varianConfigArg1 = {"Radiography Ini Path", iocshArgString};
static const iocshArg varianConfigArg2 = {"Fluorosocopy Ini Path", iocshArgString};
static const iocshArg varianConfigArg3 = {"Custom1 Ini Path", iocshArgString};
static const iocshArg varianConfigArg4 = {"Custom2 Ini Path", iocshArgString};
static const iocshArg varianConfigArg5 = {"Boot Ini Path", iocshArgInt};
static const iocshArg varianConfigArg6 = {"Check Link at Boot", iocshArgInt};
static const iocshArg varianConfigArg7 = {"maxBuffers", iocshArgInt};
static const iocshArg varianConfigArg8 = {"maxMemory", iocshArgInt};
static const iocshArg varianConfigArg9 = {"priority", iocshArgInt};
static const iocshArg varianConfigArg10 = {"stackSize", iocshArgInt};
static const iocshArg * const varianConfigArgs[] = {&varianConfigArg0,
                                                    &varianConfigArg1,
                                                    &varianConfigArg2,
                                                    &varianConfigArg3,
                                                    &varianConfigArg4,
                                                    &varianConfigArg5,
                                                    &varianConfigArg6,
                                                    &varianConfigArg7,
                                                    &varianConfigArg8,
                                                    &varianConfigArg9,
                                                    &varianConfigArg10
                                                   };
static const iocshFuncDef configvarian = {"varianConfig", 11, varianConfigArgs};
static void configvarianCallFunc(const iocshArgBuf *args)
{
    varianConfig(args[0].sval, args[1].sval, args[2].sval, args[3].sval, args[4].sval,
                 args[5].ival, args[6].ival, args[7].ival, args[8].ival, args[9].ival,
                 args[10].ival);
}

static void varianRegister(void)
{
    iocshRegister(&configvarian, configvarianCallFunc);
}

extern "C" {
    epicsExportRegistrar(varianRegister);
}
