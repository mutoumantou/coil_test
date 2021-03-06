/************************************************************************
 *
 *  Module:       NETUSBCAM_API.h 
 *  Long name:    NETUSBCAM API definition
 *
 *  Runtime Env.: Linux
 *  Author(s):    Guenter dengler 
 *  Company:      Net GmbH, Finning    http://www.net-gmbh.com
 *
 ************************************************************************/

#ifndef __NETUSBCAM_API_h__
#define __NETUSBCAM_API_h__

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
	bool			bEnabled;
	bool			bAuto;
	bool			bOnePush;
	unsigned int	nDef;
	unsigned int	nMin;
	unsigned long	nMax;	
} 
PARAM_PROPERTY, *P_PARAM_PROPERTY;

typedef struct {
	bool			bEnabled;
	bool			bAuto;
	bool			bOnePush;
	float	nDef;
	float	nMin;
	float	nMax;	
} 
PARAM_PROPERTY_f, *P_PARAM_PROPERTY_f;

// Roi 

typedef struct {
	bool	bEnabled;
	int		nXRes;
	int		nYRes;
	int		nXPos;
	int		nYPos;
} 
ROI_PROPERTY, *P_ROI_PROPERTY;

typedef struct {
	int		nXMin;
	int		nXMax;
	int		nYMin;
	int		nYMax;
} 
ROI_RANGE_PROPERTY, *P_ROI_RANGE_PROPERTY;

__attribute__((visibility("default")))
int
NETUSBCAM_SetSensor(unsigned int deviceIndex, unsigned int subAddr, unsigned int value);

__attribute__((visibility("default")))
int
NETUSBCAM_GetSensor(unsigned int deviceIndex, unsigned int subAddr, unsigned int* value);


__attribute__((visibility("default")))
int
NETUSBCAM_Init();

__attribute__((visibility("default")))
int
NETUSBCAM_Destroy(unsigned int deviceIndex);

__attribute__((visibility("default")))
int
NETUSBCAM_Open(unsigned int deviceIndex);

__attribute__((visibility("default")))
int
NETUSBCAM_Close(unsigned int deviceIndex);

__attribute__((visibility("default")))
int
NETUSBCAM_Start(unsigned int deviceIndex);

__attribute__((visibility("default")))
int
NETUSBCAM_Stop(unsigned int deviceIndex);

__attribute__((visibility("default")))
int
NETUSBCAM_GetApiVersion(char* version, unsigned int length);

__attribute__((visibility("default")))
int
NETUSBCAM_GetSerialNum(unsigned int deviceIndex, char* serial, unsigned int length);

__attribute__((visibility("default")))
int
NETUSBCAM_GetName(unsigned int deviceIndex, char* name, unsigned int length);

__attribute__((visibility("default")))
int
NETUSBCAM_SetResolution(unsigned int deviceIndex, int nXRes, int nYRes, int nXPos, int nYPos);

__attribute__((visibility("default")))

int
NETUSBCAM_UnsetResolution(unsigned int deviceIndex);

__attribute__((visibility("default")))
int
NETUSBCAM_GetResolution(unsigned int deviceIndex, int* nXRes, int* nYRes, int* nXPos, int* nYPos);

__attribute__((visibility("default")))
int
NETUSBCAM_SetBinSkip(unsigned int deviceIndex, /*int nXRes, int nYRes, */int nFaktor, int nMode);

__attribute__((visibility("default")))
int
NETUSBCAM_GetBinSkip(unsigned int deviceIndex, /*int* nXRes, int* nYRes,*/ int* nFaktor, int* nMode);

__attribute__((visibility("default")))
int
NETUSBCAM_GetModeList(unsigned int deviceIndex, unsigned int* length, unsigned int* modesList);

__attribute__((visibility("default")))
int
NETUSBCAM_GetMode(unsigned int deviceIndex, unsigned int* mode);

__attribute__((visibility("default")))
int
NETUSBCAM_SetMode(unsigned int deviceIndex, unsigned int mode);

__attribute__((visibility("default")))
int
NETUSBCAM_GetBinSkipList(unsigned int deviceIndex, unsigned int type, unsigned int* length, unsigned int* modesList);

__attribute__((visibility("default")))
int
NETUSBCAM_GetSize(unsigned int deviceIndex, int* nXRes, int* nYRes);

__attribute__((visibility("default")))
int
NETUSBCAM_GetResolutionRange(unsigned int deviceIndex,ROI_RANGE_PROPERTY *property);

__attribute__((visibility("default")))
int	NETUSBCAM_SetCamParameter(unsigned int deviceIndex,int Type,unsigned long Value);

__attribute__((visibility("default")))
int	NETUSBCAM_GetCamParameter(unsigned int deviceIndex,int Type,unsigned long* Value);

__attribute__((visibility("default")))
int	NETUSBCAM_GetCamParameterRange(unsigned int deviceIndex,int Type,PARAM_PROPERTY *property);

__attribute__((visibility("default")))
int	NETUSBCAM_GetParamAuto(unsigned int deviceIndex,int Type,int* bAuto);

__attribute__((visibility("default")))
int	NETUSBCAM_SetParamAuto(unsigned int deviceIndex,int Type,bool bAuto);

__attribute__((visibility("default")))
int	NETUSBCAM_SetParamOnePush(unsigned int deviceIndex,int Type);

__attribute__((visibility("default")))
int
NETUSBCAM_GetFWVersion(unsigned int deviceIndex,char* version, unsigned int length);
	
__attribute__((visibility("default")))
int
NETUSBCAM_GetTrigger(unsigned int deviceIndex,int* nMode);

__attribute__((visibility("default")))
int
NETUSBCAM_SetTrigger(unsigned int deviceIndex,int nMode);

__attribute__((visibility("default")))
int
NETUSBCAM_SaveToFile(unsigned int deviceIndex,const char* name);

__attribute__((visibility("default")))
int
NETUSBCAM_GetBrokenFrames(unsigned int deviceIndex,unsigned int* pnFrames);

__attribute__((visibility("default")))
int
NETUSBCAM_GetGoodFrames(unsigned int deviceIndex,unsigned int* pnFrames);

__attribute__((visibility("default")))
int	NETUSBCAM_SetExposure(unsigned int deviceIndex,float Value);

__attribute__((visibility("default")))
int	NETUSBCAM_GetExposure(unsigned int deviceIndex,float* Value);

__attribute__((visibility("default")))
int	NETUSBCAM_GetExposureRange(unsigned int deviceIndex,PARAM_PROPERTY_f *property);

__attribute__((visibility("default")))
int	NETUSBCAM_SetParamAutoDef(unsigned int deviceIndex,int Type);

// Callback
#define CALLBACK_RAW			0
#define CALLBACK_RGB			1
#define CALLBACK_Y_8			2

__attribute__((visibility("default")))
int
NETUSBCAM_SetCallback(unsigned int deviceIndex, int mode, int (*pCallbackFunc)(void* buffer, unsigned int bufferSize, void* context), void* context);

#ifdef  __cplusplus
}
#endif

#endif // __NETUSBCAM_API_h__

/******************************** EOF ***********************************/
