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
	}
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
	}
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

void	CMoveDetector::setFrame(cv::Mat	src, int chId /*= 0*/)
{
	//static unsigned int time1 = 0;
	//printf("time during : %d \n",OSA_getCurTimeInMsec() - time1);
	CV_Assert(chId	< DETECTOR_NUM);
	if( !src.empty() ){
#if 1
		cv::blur(src, frame[chId],cv::Size(5,5));
#else
		src.copyTo(frame[chId]);
#endif
		m_postDetect[chId].InitializedMD(src.cols,	src.rows>>1, src.cols);
		m_postDetect2[chId].InitializedMD(src.cols,	src.rows>>1, src.cols);
		OSA_tskSendMsg(&m_maskDetectTsk[chId], NULL, (Uint16)chId, NULL, 0);
	}
	//time1 = OSA_getCurTimeInMsec();
}

//void	CMoveDetector::setDetectRoi(cv::Rect roi, int chId/* = 0*/)
//{
//	m_detectRoi[chId] = roi;
//}


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
		static bool update_bg_model = true;
		static int  frameCount = 0;
		if(!frame[chId].empty())
		{
			Uint32 t1 = OSA_getCurTimeInMsec() ;
			frameCount++;
			if(frameCount > 500)
				update_bg_model = false;
			(*fgbg[chId])(frame[chId], fgmask[chId], update_bg_model ? -1 : 0);
			assert(fgmask[chId].channels() == 1);

			imshow("MCDETECT", fgmask[chId]);
			waitKey(1);

			OSA_printf("%s:delt_t1=%d\n",__func__, OSA_getCurTimeInMsec() - t1);

			int k;
			cv::Mat BGMask[2];
			for(k=0; k<2; k++){
				BGMask[k]= cv::Mat(fgmask[chId].rows>>1, fgmask[chId].cols, CV_8UC1, fgmask[chId].data+(k*fgmask[chId].cols*(fgmask[chId].rows>>1)) );
			}
#pragma omp parallel for
			for(k=0; k<2; k++){
				if(k==0){
					m_postDetect[chId].GetMoveDetect(BGMask[k].data, BGMask[k].cols, BGMask[k].rows, BGMask[k].cols, 5);
					m_postDetect[chId].MovTargetDetect(m_scaleX[chId],	m_scaleY[chId]);
				}else{
					m_postDetect2[chId].GetMoveDetect(BGMask[k].data, BGMask[k].cols, BGMask[k].rows, BGMask[k].cols, 5);
					m_postDetect2[chId].MovTargetDetect(m_scaleX[chId],	m_scaleY[chId]);
				}
			}
			{
				OSA_printf("%s:delt_t2=%d\n",__func__, OSA_getCurTimeInMsec() - t1);

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

				if(m_notifyFunc != NULL)
				{
					(*m_notifyFunc)(m_context, chId);
				}
		}
	}
	
}
#endif



