#include <stdio.h>
#include <stdlib.h>
#include <opencv2/video/background_segm.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "MovDetector.hpp"

using namespace cv;
using namespace std;


CMoveDetector::CMoveDetector()
{
	int	i;
	m_bExit	 = FALSE;
	for(i=0; i<DETECTOR_NUM; i++){
		m_warnRoiVec[i].clear();
		m_warnLostTarget[i].clear();
		m_warnInvadeTarget[i].clear();
		m_movTarget[i].clear();
		m_edgeTarget[i].clear();
		m_warnTarget[i].clear();
		m_warnMode[i]	= WARN_MOVEDETECT_MODE;
		m_bSelfDraw[i] = false;
		m_scaleX[i] = 1.0;
		m_scaleY[i] = 1.0;
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
	area[i] = 0;
	frameidx[i] = 0;
	processDone[i] = true;
	}
	resetFlag = false;
}

CMoveDetector::~CMoveDetector()
{
	destroy();
}

int CMoveDetector::creat(int history /*= 500*/,  float varThreshold /*= 16*/, bool bShadowDetection /*=true*/)
{
	int	i;
	initModule_video();
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


	return	0;
}

int CMoveDetector::init(LPNOTIFYFUNC	notifyFunc, void *context)
{
	int	i, result = OSA_SOK;

	m_notifyFunc	= notifyFunc;
	m_context = context;
	for(i=0; i<DETECTOR_NUM; i++)
	{
		result |= OSA_tskCreate(&m_maskDetectTsk[i], videoProcess_TskFncMaskDetect, 0, 0, 0, this);
	   OSA_printf("%s:chId=%d \n",__func__, i);
	}
	return	result;
}

int CMoveDetector::destroy()
{
	int	i,	rtn = OSA_SOK;
	m_bExit = TRUE;
	for(i=0; i<DETECTOR_NUM; i++){
		rtn |= OSA_tskDelete(&m_maskDetectTsk[i]);
	}

	for(i=0; i<DETECTOR_NUM; i++)
	{
		m_postDetect[i].DestroyMD();
		m_postDetect2[i].DestroyMD();
	}
	for(i=0; i<DETECTOR_NUM; i++){
		m_warnRoiVec[i].clear();
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
		area[i] = 0;
		frameidx[i] = 0;
		processDone[i]= false;
	}
	resetFlag = false;
	return rtn;
}

#ifdef		BGFG_CR
void	CMoveDetector::setDetectShadows(bool	bShadow,	int chId/*	= 0*/)
{
	if (fgbg[chId] != NULL){
		fgbg[chId]->setDetectShadows( bShadow);
	}
}

void	CMoveDetector::setShadowValue(int value,	int chId	/*= 0*/)
{
	if (fgbg[chId] != NULL){
		fgbg[chId]->setShadowValue( value);
	}
}

void	CMoveDetector::setHistory(int nHistory,	int chId/*	= 0*/)
{
	if (fgbg[chId] != NULL){
		fgbg[chId]->setHistory(nHistory);
	}
}

void	CMoveDetector::setVarThreshold(double varThreshold,	int chId/*	= 0*/)
{
	if (fgbg[chId] != NULL){
		fgbg[chId]->setVarThreshold(varThreshold);
	}
}

void	CMoveDetector::setVarThredGen(float	varThredGen,	int chId/*	= 0*/)
{

	if (fgbg[chId] != NULL){
		fgbg[chId]->setVarThresholdGen(varThredGen);
	}
}

void	CMoveDetector::setBackgroundRatio(float ratio,	int chId/*	= 0*/)
{
	if (fgbg[chId] != NULL){
		fgbg[chId]->setBackgroundRatio(ratio);
	}
}

void	CMoveDetector::setCompRedThred(float fct,	int chId/*	= 0*/)
{
	if (fgbg[chId] != NULL){
		fgbg[chId]->setComplexityReductionThreshold(fct);
	}
}

void	CMoveDetector::setNMixtures(int nmix,	int chId/*	= 0*/)
{
	if (fgbg[chId] != NULL){
		fgbg[chId]->setNMixtures(nmix);
	}
}

void	CMoveDetector::setVarInit(float initvalue,	int chId/*	= 0*/)
{
	if (fgbg[chId] != NULL){
		fgbg[chId]->setVarInit(initvalue);
	}
}

void	CMoveDetector::setShadowThreshold(double threshold,	int chId	/*= 0*/)
{
	if (fgbg[chId] != NULL){
		fgbg[chId]->setShadowThreshold(threshold);
	}
}

void	CMoveDetector::setNFrames(int nframes, int chId /*= 0*/)
{
	if (fgbg[chId] != NULL){
		fgbg[chId]->setDetectNFrames(nframes);
	}
}

#endif

void CMoveDetector::setFrame(cv::Mat	src ,int srcwidth , int srcheight ,int chId,int accuracy/*2*/,int inputArea/*8*/,int inputThreshold/*30*/)
{
	ASSERT( 1 == src.channels());
	
	float dstWidth ,dstHeigth;
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
	this->setROIScalXY(x,y,0);
	cv::Mat gray;
	cv::resize(src,gray, cv::Size((int)dstWidth, (int)dstHeigth));

	CV_Assert(chId	< DETECTOR_NUM);
	if( !src.empty() ){

	#if 1
		cv::blur(gray, frame[chId],cv::Size(3,3));
	#else
		src.copyTo(frame[chId]);
	#endif
		area[chId] = inputArea ;
		threshold[chId] = inputThreshold;
		if(processDone[chId]){
			processDone[chId] = false;
			m_postDetect[chId].InitializedMD(gray.cols, gray.rows>>1, gray.cols);
			m_postDetect2[chId].InitializedMD(gray.cols,gray.rows>>1, gray.cols);
			OSA_tskSendMsg(&m_maskDetectTsk[chId], NULL, (Uint16)chId, NULL, 0);
		}
	}
}


void	CMoveDetector::setWarningRoi(std::vector<cv::Point2i>	warnRoi,	int chId	/*= 0*/)
{
	CV_Assert(chId	< DETECTOR_NUM);
	int	k,	npoint	= warnRoi.size();
	CV_Assert(npoint	> 2);
	if(npoint	> 2){
		m_warnRoiVec[chId].resize(npoint);
		for(k=0; k<npoint;	k++){
			m_warnRoiVec[chId][k]	= cv::Point2i(warnRoi[k]);
		}
		m_postDetect[chId].setWarningRoi(warnRoi); // warn roi is disanormal
		m_postDetect2[chId].setWarningRoi(warnRoi);
	}else{
		OSA_printf("%s: warning	roi	point	num=%d < 3\n", __func__,	npoint);
	}
}

void	CMoveDetector::clearWarningRoi(int chId	/*= 0*/)
{
	CV_Assert(chId	< DETECTOR_NUM);
	m_warnRoiVec[chId].clear();
}

void	CMoveDetector::setTrkThred(TRK_THRED	trkThred,	int chId/*	= 0*/)
{
	CV_Assert(chId	< DETECTOR_NUM);
	m_postDetect[chId].setTrkThred(trkThred);
	m_postDetect2[chId].setTrkThred(trkThred);
}

void	CMoveDetector::setDrawOSD(cv::Mat	dispOSD, int chId /*= 0*/)
{
	CV_Assert(chId	< DETECTOR_NUM);
	disframe[chId]	= dispOSD;
}

void	CMoveDetector::setWarnMode(WARN_MODE	warnMode,	int chId /*= 0*/)
{
	CV_Assert(chId	< DETECTOR_NUM);
	m_warnMode[chId]	= warnMode;
}

void	CMoveDetector::enableSelfDraw(bool	bEnable, int chId/* = 0*/)
{
	CV_Assert(chId	< DETECTOR_NUM);
	m_bSelfDraw[chId] = bEnable;
}

void   CMoveDetector::setROIScalXY(float scaleX /*= 1.0*/, float scaleY /*= 1.0*/, int chId /*= 0*/)
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

void	CMoveDetector::getLostTarget(std::vector<TRK_RECT_INFO>	&resTarget,	int chId /*= 0*/)
{
	CV_Assert(chId	<DETECTOR_NUM);
	_copyTarget(m_warnLostTarget[chId], resTarget);
}

void	CMoveDetector::getInvadeTarget(std::vector<TRK_RECT_INFO>	&resTarget,	int chId /*= 0*/)
{
	CV_Assert(chId	<DETECTOR_NUM);
	_copyTarget(m_warnInvadeTarget[chId], resTarget);
}

void	CMoveDetector::getMoveTarget(std::vector<TRK_RECT_INFO>	&resTarget,	int chId /*= 0*/)
{
	CV_Assert(chId	<DETECTOR_NUM);
	_copyTarget(m_movTarget[chId], resTarget);
}

void	CMoveDetector::getBoundTarget(std::vector<TRK_RECT_INFO>	&resTarget,	int chId /*= 0*/)
{
	CV_Assert(chId	<DETECTOR_NUM);
	_copyTarget(m_edgeTarget[chId], resTarget);
}

void	CMoveDetector::getWarnTarget(std::vector<TRK_RECT_INFO>	&resTarget,	int chId	/*= 0*/)
{
	CV_Assert(chId	<DETECTOR_NUM);
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


void CMoveDetector::mvPause()
{
	if(!resetFlag)
		resetFlag = true;
}

#if 0
void CMoveDetector::maskDetectProcess(OSA_MsgHndl *pMsg)
{
		int chId;
		chId	=	pMsg->cmd ;
		CV_Assert(chId < DETECTOR_NUM);
		if(m_bExit)
			return;

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
			return;
		}
		bool update_bg_model = true;
		if(!frame[chId].empty())
		{
			Uint32 t1 = OSA_getCurTimeInMsec() ;
			(*fgbg[chId])(frame[chId], fgmask[chId], update_bg_model ? -1 : 0);
			OSA_printf("%s:delt_t1=%d\n",__func__, OSA_getCurTimeInMsec() - t1);

			if(m_postDetect[chId].GetMoveDetect(fgmask[chId].data, fgmask[chId].cols, fgmask[chId].rows, fgmask[chId].cols,5) )
			{
				OSA_printf("%s:delt_t2=%d\n",__func__, OSA_getCurTimeInMsec() - t1);

				if(	(m_warnMode[chId] & WARN_MOVEDETECT_MODE)	)	//move target detect
				{
					m_postDetect[chId].MovTargetDetect(m_scaleX[chId],	m_scaleY[chId]);
					m_postDetect[chId].getMoveTarget(m_movTarget[chId]);

					if(m_bSelfDraw[chId] && !disframe[chId].empty() )
					{
						int	npoint	= m_warnRoiVec[chId].size();
						for(int i=0; i<npoint; i++)
						{
							line(disframe[chId], m_warnRoiVec[chId][i], m_warnRoiVec[chId][(i+1)%npoint], cvScalar(0,0,255,255), 4, 8);
						}
						m_postDetect[chId].MovTargetDraw(disframe[chId]);
					}
				}
				else if(	(m_warnMode[chId] & WARN_INVADE_MODE)  || (m_warnMode[chId] & WARN_LOST_MODE)	 || (m_warnMode[chId] & WARN_INVAD_LOST_MODE))//lost or invade detect
				{
					m_postDetect[chId].warnTargetSelect(m_scaleX[chId],	m_scaleY[chId]);
					m_postDetect[chId].SetTargetBGFGTrk();
					m_postDetect[chId].WarnTargetBGFGTrk();
					m_postDetect[chId].TargetBGFGAnalyse();
					m_postDetect[chId].GetBGFGTarget(m_warnLostTarget[chId], m_warnInvadeTarget[chId], m_warnTarget[chId]);

					if(m_bSelfDraw[chId] && !disframe[chId].empty())
					{
						int	npoint	= m_warnRoiVec[chId].size();
						for(int i=0; i<npoint; i++)
						{
							line(disframe[chId], m_warnRoiVec[chId][i], m_warnRoiVec[chId][(i+1)%npoint], cvScalar(0,0,255,255), 4, 8);
						}
						m_postDetect[chId].DrawBGFGTarget(disframe[chId]);
					}
				}
				else if(	(m_warnMode[chId] & WARN_BOUNDARY_MODE)	)//edge target detect
				{
						m_postDetect[chId].edgeTargetDetect(m_scaleX[chId],	m_scaleY[chId]);
						m_postDetect[chId].getEdgeTarget(m_edgeTarget[chId]);

						if(m_bSelfDraw[chId] && !disframe[chId].empty())
						{
							int	npoint	= m_warnRoiVec[chId].size();
							for(int i=0; i<npoint; i++)
							{
								line(disframe[chId], m_warnRoiVec[chId][i], m_warnRoiVec[chId][(i+1)%npoint], cvScalar(0,0,255,255), 4, 8);
							}
							m_postDetect[chId].edgeTargetDraw(disframe[chId]);
						}
				}
				if(m_notifyFunc != NULL)
				{
					(*m_notifyFunc)(m_context, chId);
				}
		}
	}

}
#else
void CMoveDetector::maskDetectProcess(OSA_MsgHndl *pMsg)
{
		int chId;
		chId	=	pMsg->cmd ;
		CV_Assert(chId < DETECTOR_NUM);
		if(m_bExit){
			processDone[chId] = true;
			return;
		}
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
			processDone[chId] = true;
			return;
		}
		static bool update_bg_model = true;
		static int  frameCount = 0;
		if(!frame[chId].empty())
		{
		//	Uint32 t1 = OSA_getCurTimeInMsec() ;

#if 1
		if(frameidx[chId] < 5){
			frameidx[chId]++;		
		}
		if(frame[chId].cols != m_BKWidth[chId] || frame[chId].rows != m_BKHeight[chId]){
			if(model[chId]!= NULL)	{
				delete model[chId];
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
		}
		if(frameidx[chId] < 3)
		{
			processDone[chId] = true;
			return ;
		}
#else
		frameCount++;
		if(frameCount > 500)
			update_bg_model = false;
		(*fgbg[chId])(frame[chId], fgmask[chId], update_bg_model ? -1 : 0);
		assert(fgmask[chId].channels() == 1);
#endif
			
		//OSA_printf("%s:delt_t1=%d\n",__func__, OSA_getCurTimeInMsec() - t1);

		int k;
		cv::Mat BGMask[2];
		CPostDetect* pMVObj[2];
		for(k=0; k<2; k++){
			BGMask[k]= cv::Mat(fgmask[chId].rows>>1, fgmask[chId].cols, CV_8UC1, fgmask[chId].data+(k*fgmask[chId].cols*(fgmask[chId].rows>>1)) );
		}
		pMVObj[0] = &m_postDetect[chId];
		pMVObj[1] = &m_postDetect2[chId];
			
#pragma omp parallel for
			for(k=0; k<2; k++){
				pMVObj[k]->GetMoveDetect(BGMask[k].data, BGMask[k].cols, BGMask[k].rows, BGMask[k].cols, area[chId],5);
				pMVObj[k]->MovTargetDetect(m_scaleX[chId],	m_scaleY[chId]);
			}
			{
				//OSA_printf("%s:delt_t2=%d\n",__func__, OSA_getCurTimeInMsec() - t1);

				if(	(m_warnMode[chId] & WARN_MOVEDETECT_MODE)	)	//move target detect
				{
					int nsize1= m_postDetect[chId].m_movTargetRec.size();
					int nsize2= m_postDetect2[chId].m_movTargetRec.size();
					cv::Size offsize;
					offsize.width = 0;
					offsize.height = (int)((fgmask[chId].rows>>1)*m_scaleY[chId]);
					m_movTarget[chId].resize(nsize1+nsize2);					
					CopyTrkTarget(&m_postDetect[chId], m_movTarget[chId], nsize1, 0, cv::Size(0,0));
					CopyTrkTarget(&m_postDetect2[chId], m_movTarget[chId], nsize2, nsize1, offsize);						
				}

				if( m_notifyFunc != NULL )
				{
					(*m_notifyFunc)(m_context, chId);
				}
		}
		
	}
	if( true == resetFlag ){
		for(int i =0;i<DETECTOR_NUM; i++){
			if(model[i] != NULL){
			libvibeModel_Sequential_Free(model[i]);
			//delete model[i];
			model[i] = NULL;
		}
		m_BKWidth[i] = 0;
		m_BKHeight[i] = 0;
		threshold[i] = 0;
		m_movTarget[i].clear();
		frameidx[i] = 0;
		}
		resetFlag = false;
	}
	processDone[chId] = true;
}
#endif



