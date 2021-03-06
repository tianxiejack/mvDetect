#include <stdio.h>
#include <stdlib.h>
#include <opencv2/video/background_segm.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "MovDetector.hpp"

using namespace cv;
using namespace std;
//using namespace mv_detect;

static int _gGapFrames = 5;

namespace mv_detect{

#define	RECORD_NUM	1000	
#define	SPEEDUP_NUM	100
CMoveDetector_mv* CMoveDetector_mv::pThis = NULL;

CMoveDetector_mv::CMoveDetector_mv():m_thrOpclExit(false)
{
	int	i;
	m_bExit	 = FALSE;
	for(i=0; i<DETECTOR_NUM; i++){
		m_warnRoiVec[i].clear();
		m_warnRoiVec_bak[i].clear();
		m_warnLostTarget[i].clear();
		m_warnInvadeTarget[i].clear();
		m_movTarget[i].clear();
		m_edgeTarget[i].clear();
		m_warnTarget[i].clear();
		m_warnMode[i]	= WARN_MOVEDETECT_MODE;
		m_bSelfDraw[i] = false;
		m_scaleX[i] = 1.0;
		m_scaleY[i] = 1.0;
		m_offsetPt[i] = cv::Point(0, 0);
		m_bInterval[i] = 0;
		m_busy[i] = false;
		frameIndex[i] = 0;
		stoppingRestFlag[i] = false;
		matchThreshHoldBak = 16;
		avggray[i] = 0;
		curgray[i] = 0;
		graydelta[i] = 5;
	}
	m_notifyFunc = NULL;
	m_context = NULL;
	for(i=0; i<DETECTOR_NUM; i++){
#ifdef		BGFG_CR
		fgbg[i] = NULL;
#else
		fgbg[i] = NULL;
#endif

	model[i] = NULL;
	m_BKHeight[i] = 0;
	m_BKWidth[i] = 0;
	threshold[i] = 0;
	minArea[i] = 0;
	maxArea[i] = 0;

	statusFlag[i] = false;
	doneFlag[i]   = true;

	}
	pThis = this;
	OSA_mutexCreate(&syncSetWaringROI);
}

CMoveDetector_mv::~CMoveDetector_mv()
{
	destroy();
	OSA_mutexDelete(&syncSetWaringROI);
}

int CMoveDetector_mv::creat(int history /*= 500*/,  float varThreshold /*= 16*/, bool bShadowDetection /*=true*/)
{
    printf("\r\n Mvetect.so v1.0 --------------Build date: %s %s \r\n", __DATE__, __TIME__);

	int	i;
	//initModule_video();
	setUseOptimized(true);
	setNumThreads(4);

	for(i=0; i<DETECTOR_NUM; i++)
	{
#ifdef	BGFG_CR
		if(fgbg[i] != NULL){
			delete fgbg[i];
			fgbg[i] = NULL;
		}
		fgbg[i] = new BackgroundSubtractorMOG3;
		if(fgbg[i] == NULL)
		{
			printf( "%s:Failed to create BackgroundSubtractor.MOG3 Algorithm.",__func__ );
			assert(0);
		}
#else
//		fgbg[i] = Algorithm::create<BackgroundSubtractorMOG2>("BackgroundSubtractor.MOG2");
		if(fgbg[i] != NULL){
			delete fgbg[i];
			fgbg[i] = NULL;
		}
		fgbg[i] = new BackgroundSubtractorMOG2(history, varThreshold, bShadowDetection);
		if (fgbg[i] == NULL)
		{
			printf( "Failed to create BackgroundSubtractor.MOG2 Algorithm." );
			assert(0);
		}
#endif
	}

	m_status.clear();
	m_statusBK.clear();
	m_status.reserve(DETECTOR_NUM);
	m_statusBK.reserve(DETECTOR_NUM);
	
	OSA_thrCreate(&m_opclHandle, opencloseHandle, 0,0,0);
	return	0;
}

int CMoveDetector_mv::init(LPNOTIFYFUNC	notifyFunc, void *context)
{
	int	i, result = OSA_SOK;

	m_notifyFunc	= notifyFunc;
	m_context = context;
	for(i=0; i<DETECTOR_NUM; i++)
	{
//		result |= OSA_tskCreate(&m_maskDetectTsk[i], videoProcess_TskFncMaskDetect, 0, 0, 0, this);
		result |= Detect_threadCreate(i);
	   OSA_printf("%s:chId=%d \n",__func__, i);
	}
	return	result;
}

int CMoveDetector_mv::Detect_threadCreate(int detectId)
{
	int iRet = OSA_SOK;
	iRet = OSA_semCreate(&m_detectThrObj[detectId].procNotifySem ,1,0) ;
	OSA_assert(iRet == OSA_SOK);

	m_detectThrObj[detectId].exitProcThread = false;

	m_detectThrObj[detectId].pParent = (void*)this;

	m_detectThrObj[detectId].detectId = detectId;

	iRet = OSA_thrCreate(&m_detectThrObj[detectId].thrHandleProc, detectThreadFunc, 0, 0, &m_detectThrObj[detectId]);

	return iRet;
}

int CMoveDetector_mv::Detect_threadDestroy(int detectId)
{
	int iRet = OSA_SOK;

	m_detectThrObj[detectId].exitProcThread = true;
	OSA_semSignal(&m_detectThrObj[detectId].procNotifySem);

	iRet = OSA_thrDelete(&m_detectThrObj[detectId].thrHandleProc);

	OSA_semDelete(&m_detectThrObj[detectId].procNotifySem);

	return iRet;
}

int CMoveDetector_mv::destroy()
{
	int	i,	rtn = OSA_SOK;
	m_bExit = TRUE;
	for(i=0; i<DETECTOR_NUM; i++){
//		rtn |= OSA_tskDelete(&m_maskDetectTsk[i]);
		rtn |= Detect_threadDestroy(i);
	}

	for(i=0; i<DETECTOR_NUM; i++)
	{
		m_postDetect[i].DestroyMD();
		m_postDetect2[i].DestroyMD();
	}
	for(i=0; i<DETECTOR_NUM; i++){
		m_warnRoiVec[i].clear();
		m_warnRoiVec_bak[i].clear();
		m_warnLostTarget[i].clear();
		m_warnInvadeTarget[i].clear();
		m_movTarget[i].clear();
		m_edgeTarget[i].clear();
		m_warnTarget[i].clear();
		m_bSelfDraw[i] = false;
		m_scaleX[i] = 1.0;
		m_scaleY[i] = 1.0;
	}
	for(i=0; i<DETECTOR_NUM; i++)
	{
		#ifdef		BGFG_CR
			if(fgbg[i] != NULL)
			{
				delete fgbg[i];
				fgbg[i] = NULL;
			}
		#else
			if(fgbg[i] != NULL)
			{
				delete fgbg[i];
				fgbg[i] = NULL;
			}
		#endif
		if(model[i] != NULL){
			libvibeModel_Sequential_Free(model[i]);
			model[i] = NULL;
		}
		m_BKWidth[i] = 0;
		m_BKHeight[i] = 0;
		threshold[i] = 0;
		minArea[i] = 0;
		maxArea[i] = 0;

		statusFlag[i] = false;
		doneFlag[i] = false;
	}
	OSA_thrDelete(&m_opclHandle);
	return rtn;
}

#ifdef		BGFG_CR
void	CMoveDetector_mv::setDetectShadows(bool	bShadow,	int chId/*	= 0*/)
{
	if (fgbg[chId] != NULL){
		fgbg[chId]->setDetectShadows( bShadow);
	}
}

void	CMoveDetector_mv::setShadowValue(int value,	int chId	/*= 0*/)
{
	if (fgbg[chId] != NULL){
		fgbg[chId]->setShadowValue( value);
	}
}

void	CMoveDetector_mv::setHistory(int nHistory,	int chId/*	= 0*/)
{
	if (fgbg[chId] != NULL){
		fgbg[chId]->setHistory(nHistory);
	}
}

void	CMoveDetector_mv::setVarThreshold(double varThreshold,	int chId/*	= 0*/)
{
	if (fgbg[chId] != NULL){
		fgbg[chId]->setVarThreshold(varThreshold);
	}
}

void	CMoveDetector_mv::setVarThredGen(float	varThredGen,	int chId/*	= 0*/)
{

	if (fgbg[chId] != NULL){
		fgbg[chId]->setVarThresholdGen(varThredGen);
	}
}

void	CMoveDetector_mv::setBackgroundRatio(float ratio,	int chId/*	= 0*/)
{
	if (fgbg[chId] != NULL){
		fgbg[chId]->setBackgroundRatio(ratio);
	}
}

void	CMoveDetector_mv::setCompRedThred(float fct,	int chId/*	= 0*/)
{
	if (fgbg[chId] != NULL){
		fgbg[chId]->setComplexityReductionThreshold(fct);
	}
}

void	CMoveDetector_mv::setNMixtures(int nmix,	int chId/*	= 0*/)
{
	if (fgbg[chId] != NULL){
		fgbg[chId]->setNMixtures(nmix);
	}
}

void	CMoveDetector_mv::setVarInit(float initvalue,	int chId/*	= 0*/)
{
	if (fgbg[chId] != NULL){
		fgbg[chId]->setVarInit(initvalue);
	}
}

void	CMoveDetector_mv::setShadowThreshold(double threshold,	int chId	/*= 0*/)
{
	if (fgbg[chId] != NULL){
		fgbg[chId]->setShadowThreshold(threshold);
	}
}

void	CMoveDetector_mv::setNFrames(int nframes, int chId /*= 0*/)
{
	if (fgbg[chId] != NULL){
		fgbg[chId]->setDetectNFrames(nframes);
	}
}

#endif

void CMoveDetector_mv::setFrame(cv::Mat	src ,int chId,int accuracy/*2*/,int inputMinArea/*8*/,int inputMaxArea/*200*/,int inputThreshold/*30*/)
{	
	ASSERT( 1 == src.channels());
	ASSERT(chId >= 0 && chId < DETECTOR_NUM);
#if 0
	unsigned int frameCounttmp;
	UInt64 timeStamp = cv::getTickCount();

	memcpy(&frameCounttmp , (char*)src.data ,sizeof(unsigned int));
#endif
	if(inputThreshold < 1)
		inputThreshold = 16;
	if(inputMinArea < 100 )
		inputMinArea = 100;
	if(inputMaxArea > 100*10000)
		inputMaxArea = 100*10000;

	if(inputMinArea > inputMaxArea)
	{
		inputMinArea = 100;
		inputMaxArea = 1000000;
	}
	//UInt32 t1 = OSA_getCurTimeInMsec();
	int state = 0;
	std::vector<cv::Point>::iterator ptmp;

	int srcwidth =src.cols,  srcheight = src.rows;

	//if(model[chId] != NULL)
	//	printf("factor = %u \n", libvibeModel_Sequential_GetUpdateFactor(model[chId]));

	//if(model[chId] == NULL)
	{
		if(m_warnRoiVec[chId].size() < 3){
			std::vector<cv::Point> polyWarnRoi ;
			polyWarnRoi.resize(4);
			polyWarnRoi[0]	= cv::Point(0,0);
			polyWarnRoi[1]	= cv::Point(srcwidth,0);
			polyWarnRoi[2]	= cv::Point(srcwidth,srcheight);
			polyWarnRoi[3]	= cv::Point(0,srcheight);			
			this->setWarningRoi(polyWarnRoi,chId);	
		}else{
			int num = m_warnRoiVec[chId].size();
			for(ptmp = m_warnRoiVec[chId].begin();ptmp != m_warnRoiVec[chId].end();ptmp++){
				ASSERT((*ptmp).x <= srcwidth && (*ptmp).x >= 0);
				ASSERT((*ptmp).y <= srcheight && (*ptmp).y >= 0);
			}
		}
	}
	//printf("delta t1 = %d \n",OSA_getCurTimeInMsec() - t1);
	float dstWidth ,dstHeigth;
	cv::Mat gray;
	
	if( srcwidth*srcheight*2.0/3.0 < cv::contourArea(m_warnRoiVec[chId]) )
	{	
		if(srcwidth >= 1920)
		{
			if( 0 == accuracy ){
				dstWidth 	= 640.0;
				dstHeigth 	= 480.0;
			}else if( 1 == accuracy ){
				dstWidth 	= 720.0;
				dstHeigth 	= 576.0;
			}else if( 2 == accuracy ){
				dstWidth 	= 960.0;
				dstHeigth 	= 540.0;
			}else if( 3 == accuracy ){
				dstWidth 	= 1024.0;
				dstHeigth 	= 768.0;
			}else if( 4 == accuracy ){
				dstWidth 	= 1280.0;
				dstHeigth 	= 720.0;
			}else{
				dstWidth 	= (float)srcwidth;
				dstHeigth 	= (float)srcheight;	
			}	
		}else if(srcwidth >= 1280 && srcwidth < 1920){
			if( 0 == accuracy ){
				dstWidth 	= 480.0;
				dstHeigth 	= 320.0;
			}else if( 1 == accuracy ){
				dstWidth 	= 640.0;
				dstHeigth 	= 480.0;
			}else if( 2 == accuracy ){
				dstWidth 	= 640.0;
				dstHeigth 	= 512.0;
			}else if( 3 == accuracy ){
				dstWidth 	= 1024.0;
				dstHeigth 	= 768.0;
			}else if( 4 == accuracy ){
				dstWidth 	= 1280.0;
				dstHeigth 	= 720.0;
			}else{
				dstWidth 	= (float)srcwidth;
				dstHeigth 	= (float)srcheight;	
			}
		}else if(srcwidth<1280){
			if( 0 == accuracy ){
				dstWidth 	= 480.0;
				dstHeigth 	= 320.0;
			}else if( 1 == accuracy ){
				dstWidth 	= 640.0;
				dstHeigth 	= 480.0;
			}else if( 2 == accuracy ){
				dstWidth 	= 640.0;
				dstHeigth 	= 512.0;
			}else if( 3 == accuracy ){
				dstWidth 	= 720.0;
				dstHeigth 	= 576.0;
			}else{
				dstWidth 	= (float)srcwidth;
				dstHeigth 	= (float)srcheight;	
			}
		}		
		float x = (float)srcwidth/dstWidth;
		float y = (float)srcheight/dstHeigth;
		this->setROIScalXY(x,y,chId);
		cv::resize(src,gray, cv::Size((int)dstWidth, (int)dstHeigth));
		m_offsetPt[chId].x = 0;
		m_offsetPt[chId].y = 0;
		minArea[chId] = inputMinArea/(x*y);
		maxArea[chId] = inputMaxArea/(x*y);
	}
	else
	{
		cv::Rect boundRect;
		boundRect = boundingRect(m_warnRoiVec[chId]);
		boundRect.x = (boundRect.x&(~1));
		boundRect.y = (boundRect.y&(~1));
		boundRect.width = (boundRect.width&(~3));
		boundRect.height = (boundRect.height&(~3));
		gray.create(boundRect.height,boundRect.width,CV_8UC1);

		src(boundRect).copyTo(gray);

		float fx = 1.0;
		if(gray.cols*gray.rows > 640*480)
		{
			 fx = 2.0;
			cv::resize(gray,gray, cv::Size((int)gray.cols/2, (int)gray.rows/2));
		}

		this->setROIScalXY(fx,fx,chId);
		m_offsetPt[chId].x = boundRect.x;
		m_offsetPt[chId].y = boundRect.y;
		minArea[chId] = inputMinArea/fx;
		maxArea[chId] = inputMaxArea/fx;
//		OSA_printf("%s:boundRect(x=%d,y=%d,w=%d,h=%d)",__func__, boundRect.x, boundRect.y, boundRect.width, boundRect.height);
	}
	//printf("delta t2 = %d \n",OSA_getCurTimeInMsec() - t1);

	CV_Assert(chId	< DETECTOR_NUM);
	if( !src.empty() ){

		src.copyTo(bakOrigframe[chId]);
		
	#if 1
		cv::blur(gray, frameIn[chId],cv::Size(3,3));
	//printf("delta t3 = %d \n",OSA_getCurTimeInMsec() - t1);
	#else
		src.copyTo(frame[chId]);
	#endif

		threshold[chId] = inputThreshold;		
		m_postDetect[chId].InitializedMD(gray.cols, (gray.rows>>1)+16, gray.cols);
		m_postDetect2[chId].InitializedMD(gray.cols,(gray.rows>>1)+16, gray.cols);
	#if 0
		memcpy(frameIn[chId].data	,&frameCounttmp , sizeof(frameCounttmp));
		memcpy(frameIn[chId].data+sizeof(frameCounttmp),&timeStamp,sizeof(timeStamp)); 
	#endif
		OSA_semSignal(&m_detectThrObj[chId].procNotifySem);
		//printf("delta t4 = %d \n",OSA_getCurTimeInMsec() - t1);
	}
}

void	CMoveDetector_mv::setWarningRoi(std::vector<cv::Point2i>	warnRoi,	int chId	/*= 0*/)
{
	CV_Assert(chId	< DETECTOR_NUM);
	int	k,	npoint	= warnRoi.size();
	CV_Assert(npoint	> 2);
	if(npoint	> 2)
	{
		m_warnRoiVec[chId].resize(npoint);
		for(k=0; k<npoint;	k++){
			m_warnRoiVec[chId][k]	= cv::Point2i(warnRoi[k]);
		}

		m_postDetect[chId].setWarningRoi(warnRoi); // warn roi is disanormal
		m_postDetect2[chId].setWarningRoi(warnRoi);


		cv::Rect boundRect;
		boundRect = boundingRect(m_warnRoiVec[chId]);
		boundRect.x = (boundRect.x&(~1));
		boundRect.y = (boundRect.y&(~1));
		m_offsetPt[chId].x = boundRect.x;
		m_offsetPt[chId].y = boundRect.y;
	
		if(model[chId] != NULL)
		{
			OSA_mutexLock(&syncSetWaringROI);
			libvibeModel_Sequential_Free(model[chId]);
			model[chId] = NULL;
			OSA_mutexUnlock(&syncSetWaringROI);
		}

		m_BKWidth[chId]  = 0;
		m_BKHeight[chId] = 0;
		threshold[chId]  = 0;

		if(m_warnMode[chId] == WARN_MOVEDETECT_MODE)
			m_movTarget[chId].clear();
		else if(m_warnMode[chId] == WARN_WARN_MODE)
			m_warnTarget[chId].clear();

		
		if(m_notifyFunc != NULL)
		{
			(*m_notifyFunc)(m_context, chId);
		}
			
	}
	else
	{	
		OSA_printf("%s: warning	roi	point	num=%d < 3\n", __func__,	npoint);
	}
}

void	CMoveDetector_mv::clearWarningRoi(int chId	/*= 0*/)
{
	CV_Assert(chId	< DETECTOR_NUM);
	m_warnRoiVec[chId].clear();
	m_warnRoiVec_bak[chId].clear();
}

void	CMoveDetector_mv::setTrkThred(TRK_THRED	trkThred,	int chId/*	= 0*/)
{
	CV_Assert(chId	< DETECTOR_NUM);
	m_postDetect[chId].setTrkThred(trkThred);
	m_postDetect2[chId].setTrkThred(trkThred);
}

void	CMoveDetector_mv::setDrawOSD(cv::Mat	dispOSD, int chId /*= 0*/)
{
	CV_Assert(chId	< DETECTOR_NUM);
	disframe[chId]	= dispOSD;
}

void	CMoveDetector_mv::setWarnMode(WARN_MODE	warnMode,	int chId /*= 0*/)
{
	CV_Assert(chId	< DETECTOR_NUM);
	m_warnMode[chId]	= warnMode;
}

void	CMoveDetector_mv::enableSelfDraw(bool	bEnable, int chId/* = 0*/)
{
	CV_Assert(chId	< DETECTOR_NUM);
	m_bSelfDraw[chId] = bEnable;
}

void   CMoveDetector_mv::setROIScalXY(float scaleX /*= 1.0*/, float scaleY /*= 1.0*/, int chId /*= 0*/)
{
	CV_Assert(chId	< DETECTOR_NUM);
	m_scaleX[chId] = scaleX;
	m_scaleY[chId] = scaleY;
}

static void _copyTarget(std::vector<TRK_RECT_INFO> srcTarget, std::vector<TRK_RECT_INFO> &dstTarget)
{
	dstTarget.clear();
	int	i,	nsize = srcTarget.size();
	if(nsize >0){
		dstTarget.resize(nsize);
		for(i=0; i<nsize; i++){
			dstTarget[i] = srcTarget[i];
		}
	}
}

void	CMoveDetector_mv::getLostTarget(std::vector<TRK_RECT_INFO>	&resTarget,	int chId /*= 0*/)
{
	CV_Assert(chId	<DETECTOR_NUM);
	_copyTarget(m_warnLostTarget[chId], resTarget);
}

void	CMoveDetector_mv::getInvadeTarget(std::vector<TRK_RECT_INFO>	&resTarget,	int chId /*= 0*/)
{
	CV_Assert(chId	<DETECTOR_NUM);
	_copyTarget(m_warnInvadeTarget[chId], resTarget);
}

void	CMoveDetector_mv::getMoveTarget(std::vector<TRK_RECT_INFO>	&resTarget,	int chId /*= 0*/)
{
	CV_Assert(chId < DETECTOR_NUM);
		
	if(m_warnMode[chId] == WARN_MOVEDETECT_MODE)
		_copyTarget(m_movTarget[chId], resTarget);
	else if(m_warnMode[chId] == WARN_WARN_MODE)
		_copyTarget(m_warnTarget[chId], resTarget);
}

void	CMoveDetector_mv::getBoundTarget(std::vector<TRK_RECT_INFO>	&resTarget,	int chId /*= 0*/)
{
	CV_Assert(chId	<DETECTOR_NUM);
	_copyTarget(m_edgeTarget[chId], resTarget);
}

void	CMoveDetector_mv::getWarnTarget(std::vector<TRK_RECT_INFO>	&resTarget,	int chId	/*= 0*/)
{
	CV_Assert(chId<DETECTOR_NUM);
	_copyTarget(m_warnTarget[chId], resTarget);
}

static void CopyTrkTarget(CPostDetect *pMVObj,  std::vector<TRK_RECT_INFO> &trkTarget, int nsize, int offIdx, cv::Size offsize)
{
	int k;
	for(k=0; k<nsize; k++){
		trkTarget[offIdx+k] = pMVObj->m_movTargetRec[k];
		trkTarget[offIdx+k].targetRect.x += offsize.width;
		trkTarget[offIdx+k].targetRect.y += offsize.height;
	}
}


bool CMoveDetector_mv::isRun(int chId)
{
	if( statusFlag[chId] && !doneFlag[chId] ) 
		return true;
	else
		return false;
}

bool CMoveDetector_mv::isStopping(int chId)
{
	if( !statusFlag[chId] && !doneFlag[chId] ) 
		return true;
	else
		return false;
}

bool CMoveDetector_mv::isWait(int chId)
{
	if( !statusFlag[chId] && doneFlag[chId] )	
		return true;
	else
		return false;
}


void CMoveDetector_mv::openInside(int chId)
{
	if( isWait(chId) )
	{
		statusFlag[chId] = true;
		doneFlag[chId] = false;
		frameIndex[chId] = 0;
	}
	return;
}


void CMoveDetector_mv::closeInside(int chId)
{
	if( isRun(chId) )
	{
		statusFlag[chId] = false;
		OSA_semSignal(&m_detectThrObj[chId].procNotifySem);
	}
	return ;
}


void CMoveDetector_mv::stoppingResetInside(int chId)
{
	if( isStopping(chId) ) 
	{
		stoppingRestFlag[chId] = true;
		OSA_semSignal(&m_detectThrObj[chId].procNotifySem);
		//statusFlag[chId] = false;
		//doneFlag[chId] = true;	
	}
}


void CMoveDetector_mv::stoppingReset(int chId)
{

}


void CMoveDetector_mv::mvOpen(int chId)
{	
	if(chId >= DETECTOR_NUM  || chId < 0)
		return ;
	m_status[chId] = true;
	return ;
}


void CMoveDetector_mv::mvClose(int chId)
{
	if(chId >= DETECTOR_NUM  || chId < 0)
		return ;
	m_status[chId] = false;
	return ;
}

void  CMoveDetector_mv::speedupUpdateFactor(int chId /*= 0*/)
{
	if(model[chId]!= NULL)	{
		libvibeModel_Sequential_SetUpdateFactor(model[chId], 1);
	}
}

void  CMoveDetector_mv::recoverUpdateFactor(int chId /*= 0*/)
{
	if(model[chId]!= NULL)	{
		libvibeModel_Sequential_SetUpdateFactor(model[chId], matchThreshHoldBak);
	}
}


void  CMoveDetector_mv::setMatchingThreshold(const uint32_t matchingThreshold, int chId /*= 0*/)
{
	if(model[chId]!= NULL)	{
		matchThreshHoldBak = matchingThreshold;
		libvibeModel_Sequential_SetMatchingThreshold(model[chId], matchingThreshold);
	}
}

void  CMoveDetector_mv::setUpdateFactor(const uint32_t updateFactor, int chId /*= 0*/)
{
	if(model[chId]!= NULL)	{
		libvibeModel_Sequential_SetUpdateFactor(model[chId], updateFactor);
	}
}

static float calEuclidean(cv::Point2f pos1, cv::Point2f pos2)
{
	return (float)(sqrt((pos1.x-pos2.x)*(pos1.x-pos2.x)+(pos1.y-pos2.y)*(pos1.y-pos2.y)));
}

#define GET_AVE_OPT_POS_2(I, J)	\
		opt = (similarVal[I]+similarVal[J])/2;		\
		pt.x = floor((offsetXY[I].x+offsetXY[J].x)/2.0+0.5);	\
		pt.y = floor((offsetXY[I].y+offsetXY[J].y)/2.0+0.5);

#define GET_AVE_OPT_POS_3(optConf, I, J, K)	\
		opt = (similarVal[I]+similarVal[J]+similarVal[K])/2;		\
		pt.x = floor((offsetXY[I].x+offsetXY[J].x+offsetXY[K].x)/2.0+0.5);	\
		pt.y = floor((offsetXY[I].y+offsetXY[J].y+offsetXY[K].x)/2.0+0.5);

bool CMoveDetector_mv::getFrameMV(cv::Mat preFrame, cv::Mat curFrame, cv::Point2f  &pt)
{
	int i, j, width, height, centW, centH;
	cv::Rect	scenePos, refPos[4], curPos[4];
	cv::Mat	tmplModel[4], curImg[4];
	cv::Point2f offsetXY[4];
	float	similarVal[4], opt = 0.f;
	float  euLen[4][4];
	float optThred = 0.8, euThred = 2.0;

	width = preFrame.cols;
	height = preFrame.rows;
	if(width<=768){ //720x576
		scenePos.width = 72;
		scenePos.height = 60;
	}else if(width<=1024){//1024x768
		scenePos.width = 100;
		scenePos.height = 72;
	}else if(width<=1280){//1280x1024
		scenePos.width = 120;
		scenePos.height = 90;
	}else{//1920x1080
		scenePos.width = 156;
		scenePos.height = 96;
	}
	scenePos.x =scenePos.width*1.75;
	scenePos.y = scenePos.height*1.75;
	scenePos.x &=(~1);
	scenePos.y &=(~1);
	for(i=0; i<4; i++){
		int startx = ((i%2)==0)?(width/2-scenePos.x-scenePos.width):(width/2+scenePos.x);
		int starty = ((i/2)==0)?(height/2-scenePos.y-scenePos.height):(height/2+scenePos.y);
		refPos[i].x = (startx&(~1));
		refPos[i].y = (starty&(~1));
		refPos[i].width = scenePos.width;
		refPos[i].height = scenePos.height;

		curPos[i].x = refPos[i].x - scenePos.width*0.25;
		curPos[i].y = refPos[i].y - scenePos.height*0.25;
		curPos[i].width = scenePos.width*1.5;
		curPos[i].height = scenePos.height*1.5;
		curPos[i].x &= (~1);			curPos[i].y &= (~1);
		curPos[i].width &= (~1);	curPos[i].height &= (~1);
	}

#pragma omp parallel for
	for(i=0; i<4; i++){
		cv::Mat result;
		double minVal, maxVal;
		cv::Point minLoc,maxLoc;

		preFrame(refPos[i]).copyTo(tmplModel[i]);
		curFrame(curPos[i]).copyTo(curImg[i]);

		matchTemplate(curImg[i], tmplModel[i], result, CV_TM_CCOEFF_NORMED);
		minMaxLoc(result, &minVal, &maxVal, &minLoc, &maxLoc);

		if(maxLoc.x == 0 && maxLoc.y == 0){
			maxVal = 0.0;
			offsetXY[i].x = -1.f;
			offsetXY[i].y = -1.f;
		}else{
			offsetXY[i].x = curPos[i].x+maxLoc.x;
			offsetXY[i].y = curPos[i].y+maxLoc.y;
		}
		similarVal[i] = maxVal;
	}
	for(j=0; j<4; j++){
		for(i= j+1; i<4; i++){
			euLen[j][i] = calEuclidean(offsetXY[i], offsetXY[j]);
			euLen[i][j] = euLen[j][i];
		}
	}

	if(euLen[0][1]<euThred && similarVal[0] > optThred  && similarVal[1] > optThred){
		GET_AVE_OPT_POS_2( 0, 1);
	}else if(euLen[0][2]<euThred && similarVal[0] > optThred  && similarVal[2] > optThred){
		GET_AVE_OPT_POS_2(0, 2);
	}else if(euLen[0][3]<euThred && similarVal[0] > optThred  && similarVal[3] > optThred){
		GET_AVE_OPT_POS_2(0, 3);
	}else if(euLen[1][2]<euThred && similarVal[1] > optThred  && similarVal[2] > optThred){
		GET_AVE_OPT_POS_2(1, 2);
	}else if(euLen[1][3]<euThred && similarVal[1] > optThred  && similarVal[3] > optThred){
		GET_AVE_OPT_POS_2(1, 3);
	}else if(euLen[2][3]<euThred && similarVal[2] > optThred  && similarVal[3] > optThred){
		GET_AVE_OPT_POS_2(2, 3);
	}else{
		opt = 0.f;
		pt.x = pt.y = -1;
	}

	 if(opt>optThred){
		 return true;
	 }
	 return false;
}
#define PRINTFABLE 0

bool CMoveDetector_mv::judgeFirst(int chId)
{
	if( stoppingRestFlag[chId] )
	{
		destoryHistory(chId);
		doneFlag[chId] = true;	
		stoppingRestFlag[chId] = false ;
		return false;
	}

	if(0 == frameIndex[chId]){
		m_postDetect[chId].initWarnTarget();
		m_warnTarget[chId].clear();
	}

	if(frameIndex[chId] <= RECORD_NUM)
		frameIndex[chId]++;

	if( isWait(chId) )		
		return false;

	CV_Assert(chId < DETECTOR_NUM);
	if(m_bExit)
		return false;
	
	if(m_warnRoiVec[chId].size() == 0)
	{
		m_movTarget[chId].clear();
		m_warnLostTarget[chId].clear();
		m_warnInvadeTarget[chId].clear();
		m_warnTarget[chId].clear();
		m_edgeTarget[chId].clear();
		if(m_notifyFunc != NULL)
		{
			(*m_notifyFunc)(m_context, chId);
		}
		return false;
	}

	return true;
}

void CMoveDetector_mv::warnMoveDetectModeHandle(std::vector<TRK_RECT_INFO>& MVTarget,int chId)
{
	m_postDetect[chId].validTarget(MVTarget, m_movTarget[chId]);
	if(m_bSelfDraw[chId] && !disframe[chId].empty() )
	{
		int	npoint	= m_warnRoiVec[chId].size();
		for(int i=0; i<npoint; i++)
		{
			line(disframe[chId], m_warnRoiVec[chId][i], m_warnRoiVec[chId][(i+1)%npoint], cvScalar(0,0,255,255), 4, 8);
		}
		m_postDetect[chId].DrawWarnTarget(disframe[chId], m_movTarget[chId]);
	}
}

void CMoveDetector_mv::warnModeHandle(std::vector<TRK_RECT_INFO>& MVTarget,int chId)
{

	m_postDetect[chId].warnTargetSelect_New(MVTarget);

	m_postDetect[chId].SetTargetBGFGTrk();

	m_postDetect[chId].WarnTargetBGFGTrk_New(bakOrigframe[chId].size());

	
	//m_warnTarget[chId].clear();
	//m_warnTarget[chId] = m_postDetect[chId].m_warnTargetRec;
	//return ;
		//m_postDetect[chId].TargetBGFGAnalyse();
	m_postDetect[chId].GetBGFGTarget(m_warnLostTarget[chId], m_warnInvadeTarget[chId], m_warnTarget[chId] , frameIndex[chId]);

	m_postDetect[chId].GetMeanVar(frame[chId], m_warnTarget[chId], m_scaleX[chId],	m_scaleY[chId], cv::Size(m_offsetPt[chId].x,m_offsetPt[chId].y));

	m_postDetect[chId].WarnTargetValidAnalyse(m_warnTarget[chId],model[chId],frame[chId].data,m_scaleX[chId],m_scaleY[chId], cv::Size(m_offsetPt[chId].x,m_offsetPt[chId].y));

	if(m_bSelfDraw[chId] && !disframe[chId].empty())
	{
		int	npoint	= m_warnRoiVec[chId].size();
		for(int i=0; i<npoint; i++)
		{
			line(disframe[chId], m_warnRoiVec[chId][i], m_warnRoiVec[chId][(i+1)%npoint], cvScalar(0,0,255,255), 4, 8);
		}
		m_postDetect[chId].DrawBGFGTarget(disframe[chId]);
		m_postDetect[chId].DrawLOSTTarget(disframe[chId]);
	}
}

void CMoveDetector_mv::warnTrackModeHandle(std::vector<TRK_RECT_INFO>& MVTarget,int chId,int bRun)
{
	m_MSTrkObj[chId].Process(bakOrigframe[chId], MVTarget, bRun);
	if(m_bSelfDraw[chId] && !disframe[chId].empty())
	{
		int npoint	= m_warnRoiVec[chId].size();
		for(int i=0; i<npoint; i++)
		{
			line(disframe[chId], m_warnRoiVec[chId][i], m_warnRoiVec[chId][(i+1)%npoint], cvScalar(0,0,255,255), 4, 8);
		}
		m_MSTrkObj[chId].DrawTrkTarget(disframe[chId], disframe[chId], false);
		//m_MSTrkObj[chId].DrawTrkTarget(disframe[chId], disframe[chId], true);
	}
}

void CMoveDetector_mv::calcCurgray(int chId)
{
	unsigned int tmp = 0;
	int total = frame[chId].cols * frame[chId].rows;
	for(int i=0; i < total ; i++ )
	{
		tmp += frame[chId].data[i];
	}

	curgray[chId]  = (float)tmp/total;

	return ;
}

void CMoveDetector_mv::updateAvggray(int chId)
{
	float f = 0.5;
	if(avggray[chId] < 0.0001)
		avggray[chId] = curgray[chId];
	//else
		//avggray[chId] = avggray[chId]*f + curgray[chId]*(1-f);
	
	return ;
}


void CMoveDetector_mv::correctgray(int chId)
{
	float deta = curgray[chId] - avggray[chId];
	float index = 0;
	//printf("fabs(deta) = %f \n", fabs(deta));
	graydelta[chId] = 1.8;
	if(  fabs(deta)  > graydelta[chId]  ) 
	{
		index = fabs(deta)/deta;
		for(int i=0 ; i< frame[chId].cols*frame[chId].rows; i++)
		{
			frame[chId].data[i] = frame[chId].data[i] - index*(fabs(deta) - graydelta[chId]);
		}
	}
		
	return;
}


void CMoveDetector_mv::maskDetectProcess(int chId)
{	
	int  k;
	bool retFlag = true;	// 1 -- continue  , 0 --- fail & return

	unsigned int frameCounttmp;
	UInt64 timeStamp;
	
	retFlag = judgeFirst(chId);
	if(!retFlag)
		return ;
	
	int	bRun = false;
	if((m_bInterval[chId]%_gGapFrames)==0){
		bRun = true;
	}
	m_bInterval[chId]++;
	if(m_bInterval[chId] == (_gGapFrames+1))
		m_bInterval[chId] = 0;

			
	m_busy[chId] = true;
	
	static bool update_bg_model = true;
	static int  frameCount = 0;

	if( frameIndex[chId] < SPEEDUP_NUM )
		speedupUpdateFactor(chId);
	else
		recoverUpdateFactor(chId);
	
	if(!frameIn[chId].empty())
	{
		
#if PRINTFABLE
		int64 t1 = getTickCount();//OSA_getCurTimeInMsec() ;
#endif
#if 0
		memcpy(&frameCounttmp,frameIn[chId].data,sizeof(frameCounttmp));
		memcpy(&timeStamp,frameIn[chId].data+sizeof(frameCounttmp),sizeof(timeStamp));
#endif
		frameIn[chId].copyTo(frame[chId]);

	#if 1
		calcCurgray(chId);
		updateAvggray(chId);
		correctgray(chId);
	#endif
	
#if 1
		OSA_mutexLock(&syncSetWaringROI);
		if(frame[chId].cols != m_BKWidth[chId] || frame[chId].rows != m_BKHeight[chId]){
			if(model[chId]!= NULL)	{
				libvibeModel_Sequential_Free(model[chId]);
				model[chId]= NULL;
			}
		}
		if (model[chId] == NULL) {
			model[chId] = (vibeModel_Sequential_t*)libvibeModel_Sequential_New(threshold[chId]);	
			libvibeModel_Sequential_AllocInit_8u_C1R(model[chId], frame[chId].data, frame[chId].cols, frame[chId].rows);
			m_BKWidth[chId] = frame[chId].cols;
			m_BKHeight[chId] = frame[chId].rows;
			m_movTarget[chId].clear();
		}
	
		if(model[chId] != NULL){
			fgmask[chId] = Mat(frame[chId].rows, frame[chId].cols, CV_8UC1);

			libvibeModel_Sequential_Segmentation_8u_C1R(model[chId], frame[chId].data, fgmask[chId].data);
			libvibeModel_Sequential_Update_8u_C1R(model[chId], frame[chId].data, fgmask[chId].data);

			#if 0
			cv::Mat dispMat;
			cvtColor(fgmask[chId], dispMat, CV_GRAY2BGR);
			imshow("Binary", dispMat);
			waitKey(1);
			#endif
			
			cv::Mat element = getStructuringElement(MORPH_ELLIPSE, Size(5,5));
			cv::Mat srcMask[2];
			for(k=0; k<2; k++){
				if(k==0){
					srcMask[k] = fgmask[chId](Rect(0,0, fgmask[chId].cols, fgmask[chId].rows>>1));
				}else if(k==1){
					srcMask[k] = fgmask[chId](Rect(0, fgmask[chId].rows>>1, fgmask[chId].cols, fgmask[chId].rows>>1));
				}
			}
#pragma omp parallel for
			for(k=0; k<2; k++){
				cv::dilate(srcMask[k], srcMask[k], element);
			}
		}
		OSA_mutexUnlock(&syncSetWaringROI);
#else
		frameCount++;
		if(frameCount > 500)
			update_bg_model = false;
		(*fgbg[chId])(frame[chId], fgmask[chId], update_bg_model ? -1 : 0);
		assert(fgmask[chId].channels() == 1);
#endif
	
#if PRINTFABLE
		OSA_printf("%s:delt_t1=%f sec\n",__func__, ((getTickCount()-t1)/getTickFrequency()));
#endif
		cv::Mat BGMask[2];
		CPostDetect* pMVObj[2];
		for(k=0; k<2; k++){
			BGMask[k]= cv::Mat(((fgmask[chId].rows>>1)+16), fgmask[chId].cols, CV_8UC1, fgmask[chId].data+(k*fgmask[chId].cols*( (fgmask[chId].rows>>1)-16) ) );
		}
		pMVObj[0] = &m_postDetect[chId];
		pMVObj[1] = &m_postDetect2[chId];
		
#pragma omp parallel for
		for(k=0; k<2; k++){
			pMVObj[k]->GetMoveDetect(BGMask[k].data, BGMask[k].cols, BGMask[k].rows, BGMask[k].cols, minArea[chId],maxArea[chId],5);
			pMVObj[k]->MovTargetDetect(m_scaleX[chId],	m_scaleY[chId]);
		}
			
		{
#if PRINTFABLE
			OSA_printf("%s:delt_t2=%f sec\n",__func__, ((getTickCount()-t1)/getTickFrequency()));
#endif
			int nsize1= m_postDetect[chId].m_movTargetRec.size();
			int nsize2= m_postDetect2[chId].m_movTargetRec.size();

			std::vector<TRK_RECT_INFO>		tmpMVTarget;
			cv::Size offsize;

			offsize.width = m_offsetPt[chId].x;
			offsize.height = (int)( ((fgmask[chId].rows>>1)-16)*m_scaleY[chId]);
			offsize.height += m_offsetPt[chId].y;
			tmpMVTarget.resize(nsize1+nsize2);
			CopyTrkTarget(&m_postDetect[chId], tmpMVTarget, nsize1, 0, cv::Size(m_offsetPt[chId].x,m_offsetPt[chId].y));
			CopyTrkTarget(&m_postDetect2[chId], tmpMVTarget, nsize2, nsize1, offsize);
			m_postDetect[chId].MergeDetectRegion(tmpMVTarget);

			//m_warnTarget[chId].clear();
			//m_warnTarget[chId] = tmpMVTarget;

			if( (m_warnMode[chId] & WARN_MOVEDETECT_MODE)	)	//move target detect
			{
				warnMoveDetectModeHandle(tmpMVTarget,chId);
			}
			else if( (m_warnMode[chId] & WARN_WARN_MODE))
			{
				warnModeHandle(tmpMVTarget,chId);
			}
			else if( (m_warnMode[chId] & WARN_TRACK_MODE) )
			{
				warnTrackModeHandle(tmpMVTarget,chId,bRun);
			}
		#if 0
			printf("************before callback***************\n");
			printf(" frameCount : %d, during serFrame2callback time:%f\n",frameCounttmp,(cv::getTickCount() - timeStamp)*1000/cv::getTickFrequency());
			printf("------------------------------------\n");
		#endif		
			if( m_notifyFunc != NULL )
			{
				(*m_notifyFunc)(m_context, chId);
			}
#if PRINTFABLE
			OSA_printf("%s:delt_t3=%f sec\n",__func__, ((getTickCount()-t1)/getTickFrequency()));
#endif
		}
	}

	if( isStopping(chId) )
	{
		destoryHistory(chId);
		if( m_notifyFunc != NULL )
		{
			(*m_notifyFunc)(m_context, chId);
		}	
		doneFlag[chId] = true;	
	}
	m_busy[chId] = false;
}



void CMoveDetector_mv::destoryHistory(int chId)
{
	if(model[chId] != NULL){
		libvibeModel_Sequential_Free(model[chId]);
		model[chId] = NULL;
	}
	m_BKWidth[chId]  = 0;
	m_BKHeight[chId] = 0;
	threshold[chId]  = 0;
	
	if(m_warnMode[chId] == WARN_MOVEDETECT_MODE)
		m_movTarget[chId].clear();
	else if(m_warnMode[chId] == WARN_WARN_MODE)
		m_warnTarget[chId].clear();
}


void CMoveDetector_mv::mainProcTsk(DETECT_ProcThrObj *muv)
{
	OSA_printf("%s: Detect Proc Tsk[%d] Is Entering...\n",__func__, muv->detectId);
	CMoveDetector_mv *pUse = (CMoveDetector_mv *)muv->pParent;
	while(muv->exitProcThread ==  false)
	{

		OSA_semWait(&muv->procNotifySem, OSA_TIMEOUT_FOREVER);

		maskDetectProcess(muv->detectId);

	}
	OSA_printf("%s: Detect Proc Tsk[%d] Is Exit...\n",__func__, muv->detectId);
}


void* CMoveDetector_mv::opencloseHandle(void* p)
{
	int i;
	static int num = 0;
	struct timeval tv;

	while(pThis->m_thrOpclExit == false)
	{			
		for(i=0;i<DETECTOR_NUM;i++){
			if( pThis->m_status[i] ){
				if( !pThis->isRun(i) )
					pThis->openInside(i);	
			}else{
				if( !pThis->isWait(i) )
					pThis->closeInside(i);
			}
		}
		tv.tv_sec = 0;
		tv.tv_usec = 10*1000;
		select(0, NULL, NULL, NULL, &tv);
	}
	return NULL;
}


}





