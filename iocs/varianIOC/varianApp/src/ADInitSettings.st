program ADInitSettings ("A=SIM:Qi2")
#define AD_INIT_SETTINGS_VERSION 1.2
/********************************************************************************************
 *	Generic program to initialize area detector pipeline settings, 
 *  and reset tiff file counter on file name change.
 *	
 *	by Alireza Panna, NIH/NHLBI/IPL
 *	21 November, 2015
 *	A 			= prefix for camera ioc.
 *  
 *  __log__ 	= 11/21/2015: (AP) Version 1.0 created
 *				= 11/22/2015: (AP) added reset filenum functionality for other AD file types
 *				= 01/03/2016: (AP) changed AD Pipeline so that output flow is as follows:
 *							       Image1->ROI1->PROC1->((ROI2->STATS1) && FileNetCDF1 && FileTIFF1 
 *							       && FileJPEG1 && FileNexus1 && FileMagick1 && FileHDF1)
 *				= 01/12/2016: (AP) Removed unnecessary includes
 *				= 06/17/2016: (AP) Added Remove outliers plugin to AD Pipeline. Output flow is:
 *								   Image1->ROI1->RO1->PROC1->((ROI2->STATS1) && FileNetCDF1 && FileTIFF1
 *								   && FileJPEG1 && FileNexus1 && FileMagick1 && FileHDF1)
 *				= 07/10/2016: (AP) Added Transform plugin to AD Pipeline. Output flow is:
 *								   Image1->ROI1->RO1->TRANS1->PROC1->((ROI2->STATS1) && FileNetCDF1 && FileTIFF1
 *								   && FileJPEG1 && FileNexus1 && FileMagick1 && FileHDF1)
 *	__to-do__  	= 		  	
 *********************************************************************************************/ 

/*===========================Option statements========================*/
option +r; /* allows running more than one copy with different macros */
option -c; /* dont wait for all channel connections*/
option -l; /* use C line numbers in error messages etc. */
option +d; /* Turn on run time debug messages */
option +w; /* Suppress snc compiler warnings */
option -s; /* dont run in safe mode */

/* to use PV macro */
#include "seqPVmacros.h" 
/* Escape to C code for C includes using %% */
%% #include <string.h>

#if defined(_WIN32)
%% #include <Windows.h>
#endif
#if defined(__unix__) || defined(__unix) || defined(unix)
%% #include <unistd.h>								      // POSIX flags 					 
#endif

/*======================================assignments/declarations=====================================================*/
PV (short,		camCallbacks,			"{A}:cam1:ArrayCallbacks",			NoMon);			/* main AD cam callbacks */
PV (short,		image1Callbacks,		"{A}:image1:EnableCallbacks",		NoMon);			/* image1 callbacks */
PV (short,		image1ArrayCallbacks,	"{A}:image1:ArrayCallbacks",		NoMon);			/* image1 array callbacks */
PV (short,  	process1Callbacks,		"{A}:Proc1:EnableCallbacks", 		NoMon);			/* process1 callbacks */
PV (short,  	process1ArrayCallbacks,	"{A}:Proc1:ArrayCallbacks", 		NoMon);			/* process1 array callbacks */
PV (short,  	roi1Callbacks,			"{A}:ROI1:EnableCallbacks", 		NoMon);			/* roi1 callbacks */
PV (short,  	roi1ArrayCallbacks,		"{A}:ROI1:ArrayCallbacks", 			NoMon);			/* roi1 array callbacks */
PV (short,  	roi2Callbacks,			"{A}:ROI2:EnableCallbacks", 		NoMon);			/* roi2 callbacks */
PV (short,  	roi2ArrayCallbacks,		"{A}:ROI2:ArrayCallbacks", 			NoMon);			/* roi2 array callbacks */
PV (short,  	stats1Callbacks,		"{A}:Stats1:EnableCallbacks", 		NoMon);			/* stats1 callbacks */
PV (short,  	stats1ArrayCallbacks,	"{A}:Stats1:ArrayCallbacks", 		NoMon);			/* stats1 array callbacks */
PV (short,  	tiff1Callbacks,			"{A}:TIFF1:EnableCallbacks", 		NoMon);			/* tiff1 callbacks */
PV (short,  	tiff1ArrayCallbacks,	"{A}:TIFF1:ArrayCallbacks", 		NoMon);			/* tiff1 array callbacks */
PV (short,  	hdf1ArrayCallbacks,		"{A}:HDF1:ArrayCallbacks", 			NoMon);			/* hdf1 array callbacks */
PV (short,  	jpeg1ArrayCallbacks,	"{A}:JPEG1:ArrayCallbacks", 		NoMon);			/* jpeg1 array callbacks */
PV (short,  	nexus1ArrayCallbacks,	"{A}:Nexus1:ArrayCallbacks", 		NoMon);			/* nexus1 array callbacks */
PV (short,  	netcdf1ArrayCallbacks,	"{A}:netCDF1:ArrayCallbacks", 		NoMon);			/* netcdf1 array callbacks */
PV (short,  	magick1ArrayCallbacks,	"{A}:Magick1:ArrayCallbacks", 		NoMon);			/* magick1 array callbacks */
PV (short,  	ro1Callbacks,			"{A}:RO1:EnableCallbacks", 		    NoMon);			/* ro1 (remove outliers) callbacks */
PV (short,  	ro1ArrayCallbacks,		"{A}:RO1:ArrayCallbacks", 			NoMon);			/* ro1 (remove outliers) array callbacks */
PV (short,  	trans1Callbacks,		"{A}:Trans1:EnableCallbacks", 		NoMon);			/* trans1 (transform array) callbacks */
PV (short,  	trans1ArrayCallbacks,	"{A}:Trans1:ArrayCallbacks", 		NoMon);			/* trans1 (transform array) array callbacks */

PV (string, 	proc1Port,				"{A}:Proc1:NDArrayPort", 			NoMon);			/* process1 Port */
PV (string, 	roi1Port,				"{A}:ROI1:NDArrayPort", 			NoMon);			/* roi1 Port */
PV (string, 	roi2Port,				"{A}:ROI2:NDArrayPort", 			NoMon);			/* roi2 Port */
PV (string, 	stats1Port,				"{A}:Stats1:NDArrayPort", 			NoMon);			/* stats1 Port */
PV (string, 	tiff1Port,				"{A}:TIFF1:NDArrayPort", 			NoMon);			/* tiff1 Port */
PV (string, 	hdf1Port,				"{A}:HDF1:NDArrayPort", 			NoMon);			/* hdf1 Port */
PV (string, 	jpeg1Port,				"{A}:JPEG1:NDArrayPort", 			NoMon);			/* jpeg1 Port */
PV (string, 	nexus1Port,				"{A}:Nexus1:NDArrayPort", 			NoMon);			/* nexus1 Port */
PV (string, 	netcdf1Port,			"{A}:netCDF1:NDArrayPort", 			NoMon);			/* netcdf1 Port */
PV (string, 	magick1Port,			"{A}:Magick1:NDArrayPort", 			NoMon);			/* magick1 Port */
PV (string, 	ro1Port,				"{A}:RO1:NDArrayPort", 				NoMon);			/* ro1 (remove outliers) Port */
PV (string, 	trans1Port,				"{A}:Trans1:NDArrayPort", 			NoMon);			/* trans1 (transform array) Port */

PV (short,  	tiff1autoSave,			"{A}:TIFF1:AutoSave", 				NoMon);			/* tiff1 auto-save */
PV (short,  	tiff1Inc,				"{A}:TIFF1:AutoIncrement", 			NoMon);			/* tiff1 auto-increment */
PV (double,		tiff1fileNum,			"{A}:TIFF1:FileNumber",				NoMon);			/* tiff1 file number */
PV (short,  	hdf1autoSave,			"{A}:HDF1:AutoSave", 				NoMon);			/* hdf1 auto-save */
PV (short,  	hdf1Inc,				"{A}:HDF1:AutoIncrement", 			NoMon);			/* hdf1 auto-increment */
PV (double,		hdf1fileNum,			"{A}:HDF1:FileNumber",				NoMon);			/* hdf1 file number */
PV (short,  	jpeg1autoSave,			"{A}:JPEG1:AutoSave", 				NoMon);			/* jpeg1 auto-save */
PV (short,  	jpeg1Inc,				"{A}:JPEG1:AutoIncrement", 			NoMon);			/* jpeg1 auto-increment */
PV (double,		jpeg1fileNum,			"{A}:JPEG1:FileNumber",				NoMon);			/* jpeg1 file number */
PV (short,  	nexus1autoSave,			"{A}:Nexus1:AutoSave", 				NoMon);			/* nexus1 auto-save */
PV (short,  	nexus1Inc,				"{A}:Nexus1:AutoIncrement", 		NoMon);			/* nexus1 auto-increment */
PV (double,		nexus1fileNum,			"{A}:Nexus1:FileNumber",			NoMon);			/* nexus1 file number */
PV (short,  	netcdf1autoSave,		"{A}:netCDF1:AutoSave", 			NoMon);			/* netcdf1 auto-save */
PV (short,  	netcdf1Inc,				"{A}:netCDF1:AutoIncrement", 		NoMon);			/* netcdf1 auto-increment */
PV (double,		netcdf1fileNum,			"{A}:netCDF1:FileNumber",			NoMon);			/* netcdf1 file number */
PV (short,  	magick1autoSave,		"{A}:Magick1:AutoSave", 			NoMon);			/* magick1 auto-save */
PV (short,  	magick1Inc,				"{A}:Magick1:AutoIncrement", 		NoMon);			/* magick1 auto-increment */
PV (double,		magick1fileNum,			"{A}:Magick1:FileNumber",			NoMon);			/* magick1 file number */
PV (short,  	proc1DataType,    		"{A}:Proc1:DataTypeOut", 			NoMon);			/* image data type */

char   tiff1Name[256];
assign tiff1Name to "{A}:TIFF1:FileName";
monitor tiff1Name;   

char tiff1FileTemplate[256];
assign tiff1FileTemplate to "{A}:TIFF1:FileTemplate";

evflag tiff1NameChange;
sync tiff1Name tiff1NameChange;

char   hdf1Name[256];
assign hdf1Name to "{A}:HDF1:FileName";
monitor hdf1Name;

char hdf1FileTemplate[256];
assign hdf1FileTemplate to "{A}:HDF1:FileTemplate";

evflag hdf1NameChange;
sync hdf1Name hdf1NameChange;

char   jpeg1Name[256];
assign jpeg1Name to "{A}:JPEG1:FileName";
monitor jpeg1Name;

char jpeg1FileTemplate[256];
assign jpeg1FileTemplate to "{A}:JPEG1:FileTemplate";

evflag jpeg1NameChange;
sync jpeg1Name jpeg1NameChange;

char   netcdf1Name[256];
assign netcdf1Name to "{A}:netCDF1:FileName";
monitor netcdf1Name;

char netcdf1FileTemplate[256];
assign netcdf1FileTemplate to "{A}:netCDF1:FileTemplate";

evflag netcdf1NameChange;
sync netcdf1Name netcdf1NameChange;

char   nexus1Name[256];
assign nexus1Name to "{A}:Nexus1:FileName";
monitor nexus1Name;

char nexus1FileTemplate[256];
assign nexus1FileTemplate to "{A}:Nexus1:FileTemplate";

evflag nexus1NameChange;
sync nexus1Name nexus1NameChange;

char   magick1Name[256];
assign magick1Name to "{A}:Magick1:FileName";
monitor magick1Name;

char magick1FileTemplate[256];
assign magick1FileTemplate to "{A}:Magick1:FileTemplate";

evflag magick1NameChange;
sync magick1Name magick1NameChange;

/*====================State Sets====================*/
ss adInit
{
	state init
	{
		/* This will be performed once by default every time the sequence starts*/
		when (1) {
			/* Enable all callbacks first */
			PVPUT(camCallbacks, 1);
			PVPUT(image1Callbacks, 1);
			PVPUT(image1ArrayCallbacks, 1);
			PVPUT(process1Callbacks, 1);
			PVPUT(process1ArrayCallbacks, 1);
			PVPUT(roi1Callbacks, 1);
			PVPUT(roi1ArrayCallbacks, 1);
			PVPUT(roi2Callbacks, 1);
			PVPUT(roi2ArrayCallbacks, 1);
			PVPUT(stats1Callbacks, 1);
			PVPUT(stats1ArrayCallbacks, 1);
			PVPUT(tiff1Callbacks, 1);
			PVPUT(tiff1ArrayCallbacks, 1);
			PVPUT(hdf1ArrayCallbacks, 1);
			PVPUT(jpeg1ArrayCallbacks, 1);
			PVPUT(netcdf1ArrayCallbacks, 1);
			PVPUT(nexus1ArrayCallbacks, 1);
			PVPUT(magick1ArrayCallbacks, 1);
			PVPUT(ro1Callbacks, 1);
			PVPUT(ro1ArrayCallbacks, 1);
			PVPUT(trans1Callbacks, 1);
			PVPUT(trans1ArrayCallbacks, 1);

			/* Format the ports for the AD pipeline */
			PVPUTSTR(ro1Port, "ROI1");
			PVPUTSTR(trans1Port, "RO1");
			PVPUTSTR(proc1Port, "TRANS1");
			PVPUTSTR(roi2Port, "PROC1");
			PVPUTSTR(stats1Port, "ROI2");
			PVPUTSTR(tiff1Port, "PROC1");
			PVPUTSTR(hdf1Port, "PROC1");
			PVPUTSTR(jpeg1Port, "PROC1");
			PVPUTSTR(netcdf1Port, "PROC1");
			PVPUTSTR(nexus1Port, "PROC1");
			PVPUTSTR(magick1Port, "PROC1");

			/* output datatype -> float32 */
			PVPUT(proc1DataType, 6);

			/* tiff file format initialization */
			PVPUT(tiff1autoSave, 1);
			PVPUT(tiff1Inc, 1);
			sprintf(tiff1FileTemplate, "%%s%%s_%%3.3d.tif");
        	pvPut(tiff1FileTemplate);	
			
			/* hdf file format initialization */
			PVPUT(hdf1autoSave, 1);
			PVPUT(hdf1Inc, 1);
			sprintf(hdf1FileTemplate, "%%s%%s_%%3.3d.h5");
        	pvPut(hdf1FileTemplate);

			/* jpeg file format initialization */
			PVPUT(jpeg1autoSave, 1);
			PVPUT(jpeg1Inc, 1);
			sprintf(jpeg1FileTemplate, "%%s%%s_%%3.3d.jpg");
        	pvPut(jpeg1FileTemplate);	
			
			/* nexus file format initialization */
			PVPUT(nexus1autoSave, 1);
			PVPUT(nexus1Inc, 1);
			sprintf(nexus1FileTemplate, "%%s%%s_%%3.3d.h5");
        	pvPut(nexus1FileTemplate);	
			
			/* netcdf file format initialization */
			PVPUT(netcdf1autoSave, 1);
			PVPUT(netcdf1Inc, 1);
			sprintf(netcdf1FileTemplate, "%%s%%s_%%3.3d.nc");
        	pvPut(netcdf1FileTemplate);

			/* magick file format initialization */
			PVPUT(magick1autoSave, 1);
			PVPUT(magick1Inc, 1);
			sprintf(magick1FileTemplate, "%%s%%s_%%3.3d.tif");
        	pvPut(magick1FileTemplate);				
			
		} state reset
	}

	state reset
	{
		/* reset filenumber via testing event flag linked to filename PV */
		entry {
			efClear(tiff1NameChange);
			efClear(jpeg1NameChange);
			efClear(hdf1NameChange);
			efClear(nexus1NameChange);
			efClear(magick1NameChange);
			efClear(netcdf1NameChange);
		}
		
		when (efTestAndClear(tiff1NameChange)) {
			PVPUT(tiff1fileNum, 0);
		} state reset
		
		when (efTestAndClear(jpeg1NameChange)) {
			PVPUT(jpeg1fileNum, 0);
		} state reset
		
		when (efTestAndClear(hdf1NameChange)) {
			PVPUT(hdf1fileNum, 0);
		} state reset
		
		when (efTestAndClear(nexus1NameChange)) {
			PVPUT(nexus1fileNum, 0);
		} state reset
		
		when (efTestAndClear(netcdf1NameChange)) {
			PVPUT(netcdf1fileNum, 0);
		} state reset
		
		when (efTestAndClear(magick1NameChange)) {
			PVPUT(magick1fileNum, 0);
		} state reset
	} 
}