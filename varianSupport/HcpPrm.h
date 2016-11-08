#ifndef INC_HCPPRM_H
#define INC_HCPPRM_H  (1)

//----------------------------------------------------------------------
//  CHcpPrm
//  -------
//  Class to store any integer or pointer type that may be passed
//  as a parameter on an intermodule call within the VirtualCP.
//
//  A private union UHcpPrm defines a member for each supported type.
//  This union must be updated whenever a new parameter type is added.
//  Note that the union is NOT being used to perform type conversions:
//  whenever a value is stored via a given member, it is then always
//  retrieved from the same member.
//  
//  The public interface defines two member functions for each type:
//
//    1) the assignment operator "operator=" is overloaded to allow
//       a source value of a particular type to be assigned into an
//       CHcpPrm object. This works by copying the value into the
//       corresponding union member. The original code had (int) casts
//       applied to the source values - these must be removed so that
//       the proper overloaded operator can be selected. This file
//       defines a preprocessor symbol HCP_PRM_CAST(X) so that these
//       casts can be disabled by editing them to be HCP_PRM_CAST(int)
//       (ultimately, these casts could just be deleted).
//
//    2) a separate cast operator (operator WORD * for example),
//       is defined to allow a CHcpPrm to be cast into a destination
//       value of a particular type. This works by copying the value
//       out of the corresponding union member. Here, the original
//       casts must be retained so that the proper cast operator can be
//       selected. This reduces the number of changes to the original
//       code, but it is also necessary because C++ does not allow
//       the assignment operator to be overloaded based on the type
//       of the destination variable.
//
//----------------------------------------------------------------------

// Setting the following conditional to #if (0) reverts the build to
// the original method of defining HcpPrm as an array of int.
#if (1)
// Symbol for disabling casts, for example HCP_PRM_CAST(int)
#define HCP_PRM_CAST(X)

union UHcpPrm
{
    int                             prmInt;
    int                             *prmIntPtr;
    char                            *prmCharPtr;
    WORD                            *prmWordPtr;
    WORD                            **prmWordPtrPtr;
    double                          *prmDoublePtr;
    void                            *prmVoidPtr;
    struct ConfigDataStruct         *prmConfigDataStructPtr;
    struct SAcqPrms                 *prmSAcqPrmsPtr;
    struct SAnalogOffsetInfo        *prmSAnalogOffsetInfoPtr;
    struct SAnalogOffsetParams      *prmSAnalogOffsetParamsPtr;
    struct SCalAcqData              *prmSCalAcqDataPtr;
    struct SCalCtrl                 *prmSCalCtrlPtr;
    struct SCalInfo                 *prmSCalInfoPtr;
    struct SCalLimits               *prmSCalLimitsPtr;
    struct SCheckLink               *prmSCheckLinkPtr;
    struct SCorrectImage            *prmSCorrectImagePtr;
    struct SCorrections             *prmSCorrectionsPtr;
    struct SFluoroModePrms          *prmSFluoroModePrmsPtr;
    struct SFluoroSysPrms           *prmSFluoroSysPrmsPtr;
    struct SGainScaling             *prmSGainScalingPtr;
    struct SHwConfig                *prmSHwConfigPtr;
    struct SModeInfo                *prmSModeInfoPtr;
    struct SModeInfoEx              *prmSModeInfoExPtr;
    struct SOpenReceptorLink        *prmSOpenReceptorLinkPtr;
    struct SQueryErrorInfo          *prmSQueryErrorInfoPtr;
    struct SRecSpecific             *prmSRecSpecificPtr;
    struct SSysInfo                 *prmSSysInfoPtr;
    struct SSysMode                 *prmSSysModePtr;
    struct SRadChkResult            *prmSRadChkResultPtr;
    union  UQueryProgInfo           *prmUQueryProgInfoPtr;
};

class CHcpPrm
{
     UHcpPrm  prmValue;
public:
// Store value functions
    int operator=(int newValue)
    {
        return (prmValue.prmInt = newValue);
    };
    int* operator=(int* newValue)
    {
        return (prmValue.prmIntPtr = newValue);
    };
    char* operator=(char* newValue)
    {
        return (prmValue.prmCharPtr = newValue);
    };
    WORD* operator=(WORD* newValue)
    {
        return (prmValue.prmWordPtr = newValue);
    };
    WORD** operator=(WORD** newValue)
    {
        return (prmValue.prmWordPtrPtr = newValue);
    };
    double* operator=(double* newValue)
    {
        return (prmValue.prmDoublePtr = newValue);
    };
    void* operator=(void* newValue)
    {
        return (prmValue.prmVoidPtr = newValue);
    };
    ConfigDataStruct* operator=(ConfigDataStruct* newValue)
    {
        return (prmValue.prmConfigDataStructPtr = newValue);
    };
    SAcqPrms* operator=(SAcqPrms* newValue)
    {
        return (prmValue.prmSAcqPrmsPtr = newValue);
    };
    SAnalogOffsetInfo* operator=(SAnalogOffsetInfo* newValue)
    {
        return (prmValue.prmSAnalogOffsetInfoPtr = newValue);
    };
    SAnalogOffsetParams* operator=(SAnalogOffsetParams* newValue)
    {
        return (prmValue.prmSAnalogOffsetParamsPtr = newValue);
    };
    SCalAcqData* operator=(SCalAcqData* newValue)
    {
        return (prmValue.prmSCalAcqDataPtr = newValue);
    };
    SCalCtrl* operator=(SCalCtrl* newValue)
    {
        return (prmValue.prmSCalCtrlPtr = newValue);
    };
    SCalInfo* operator=(SCalInfo* newValue)
    {
        return (prmValue.prmSCalInfoPtr = newValue);
    };
    SCalLimits* operator=(SCalLimits* newValue)
    {
        return (prmValue.prmSCalLimitsPtr = newValue);
    };
    SCheckLink* operator=(SCheckLink* newValue)
    {
        return (prmValue.prmSCheckLinkPtr = newValue);
    };
    SCorrectImage* operator=(SCorrectImage* newValue)
    {
        return (prmValue.prmSCorrectImagePtr = newValue);
    };
    SCorrections* operator=(SCorrections* newValue)
    {
        return (prmValue.prmSCorrectionsPtr = newValue);
    };
    SFluoroModePrms* operator=(SFluoroModePrms* newValue)
    {
        return (prmValue.prmSFluoroModePrmsPtr = newValue);
    };
    SFluoroSysPrms* operator=(SFluoroSysPrms* newValue)
    {
        return (prmValue.prmSFluoroSysPrmsPtr = newValue);
    };
    SGainScaling* operator=(SGainScaling* newValue)
    {
        return (prmValue.prmSGainScalingPtr = newValue);
    };
    SHwConfig* operator=(SHwConfig* newValue)
    {
        return (prmValue.prmSHwConfigPtr = newValue);
    };
    SModeInfo* operator=(SModeInfo* newValue)
    {
        return (prmValue.prmSModeInfoPtr = newValue);
    };
    SModeInfoEx* operator=(SModeInfoEx* newValue)
    {
        return (prmValue.prmSModeInfoExPtr = newValue);
    };
    SOpenReceptorLink* operator=(SOpenReceptorLink* newValue)
    {
        return (prmValue.prmSOpenReceptorLinkPtr = newValue);
    };
    SQueryErrorInfo* operator=(SQueryErrorInfo* newValue)
    {
        return (prmValue.prmSQueryErrorInfoPtr = newValue);
    };
    SRecSpecific* operator=(SRecSpecific* newValue)
    {
        return (prmValue.prmSRecSpecificPtr = newValue);
    };
    SSysInfo* operator=(SSysInfo* newValue)
    {
        return (prmValue.prmSSysInfoPtr = newValue);
    };
    SSysMode* operator=(SSysMode* newValue)
    {
        return (prmValue.prmSSysModePtr = newValue);
    };
    SRadChkResult* operator=(SRadChkResult* newValue)
    {
        return (prmValue.prmSRadChkResultPtr = newValue);
    };
    UQueryProgInfo* operator=(UQueryProgInfo* newValue)
    {
        return (prmValue.prmUQueryProgInfoPtr = newValue);
    };
// Retrieve value functions
    operator int ()
    {
        return prmValue.prmInt;
    };
    operator int* ()
    {
        return prmValue.prmIntPtr;
    };
    operator char* ()
    {
        return prmValue.prmCharPtr;
    };
    operator WORD* ()
    {
        return prmValue.prmWordPtr;
    };
    operator WORD** ()
    {
        return prmValue.prmWordPtrPtr;
    };
    operator double* ()
    {
        return prmValue.prmDoublePtr;
    };
    operator void* ()
    {
        return prmValue.prmVoidPtr;
    };
    operator ConfigDataStruct* ()
    {
        return prmValue.prmConfigDataStructPtr;
    };
    operator SAcqPrms* ()
    {
        return prmValue.prmSAcqPrmsPtr;
    };
    operator SAnalogOffsetInfo* ()
    {
        return prmValue.prmSAnalogOffsetInfoPtr;
    };
    operator SAnalogOffsetParams* ()
    {
        return prmValue.prmSAnalogOffsetParamsPtr;
    };
    operator SCalAcqData* ()
    {
        return prmValue.prmSCalAcqDataPtr;
    };
    operator SCalCtrl* ()
    {
        return prmValue.prmSCalCtrlPtr;
    };
    operator SCalInfo* ()
    {
        return prmValue.prmSCalInfoPtr;
    };
    operator SCalLimits* ()
    {
        return prmValue.prmSCalLimitsPtr;
    };
    operator SCheckLink* ()
    {
        return prmValue.prmSCheckLinkPtr;
    };
    operator SCorrectImage* ()
    {
        return prmValue.prmSCorrectImagePtr;
    };
    operator SCorrections* ()
    {
        return prmValue.prmSCorrectionsPtr;
    };
    operator SFluoroModePrms* ()
    {
        return prmValue.prmSFluoroModePrmsPtr;
    };
    operator SFluoroSysPrms* ()
    {
        return prmValue.prmSFluoroSysPrmsPtr;
    };
    operator SGainScaling* ()
    {
        return prmValue.prmSGainScalingPtr;
    };
    operator SHwConfig* ()
    {
        return prmValue.prmSHwConfigPtr;
    };
    operator SModeInfo* ()
    {
        return prmValue.prmSModeInfoPtr;
    };
    operator SModeInfoEx* ()
    {
        return prmValue.prmSModeInfoExPtr;
    };
    operator SOpenReceptorLink* ()
    {
        return prmValue.prmSOpenReceptorLinkPtr;
    };
    operator SQueryErrorInfo* ()
    {
        return prmValue.prmSQueryErrorInfoPtr;
    };
    operator SRecSpecific* ()
    {
        return prmValue.prmSRecSpecificPtr;
    };
    operator SSysInfo* ()
    {
        return prmValue.prmSSysInfoPtr;
    };
    operator SSysMode* ()
    {
        return prmValue.prmSSysModePtr;
    };
    operator SRadChkResult* ()
    {
        return prmValue.prmSRadChkResultPtr;
    };
    operator UQueryProgInfo* ()
    {
        return prmValue.prmUQueryProgInfoPtr;
    };
};

#else
// The following code is selected by setting #if (0) up at the top of
// this file. It reverts the build to the original method of defining
// HcpPrm as an array of int. This is intended only for test purposes.
// For production builds, the new CHcpPrm class is to be used.
//
// Any definitions of HCP_PRM_CAST(int) become (int) casts
//
#define HCP_PRM_CAST(X)  (X)
// We use the preprocessor here instead of a typedef so that the
// HcpPrm variable type is reported as "int". 
#define CHcpPrm  int
// The following is a trick - HcpStructOk was originally defined as:
//     HcpStructOk(int strIdx, int strPtr)
// This worked because strPtr was being passed as an int in the HcpPrm array.
// Now, we have had to make this a proper pointer:
//     HcpStructOk(int strIdx, void* strPtr)
// So, we use the preprocessor to insert a (void*) cast on this value. 
#define HcpStructOk(STRIDX,STRPTR) HcpStructOk(STRIDX, (void*)STRPTR)
#endif

#endif
