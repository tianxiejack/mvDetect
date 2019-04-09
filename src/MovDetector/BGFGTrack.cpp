#include <stdio.h>
#include <stdlib.h>
#include "BGFGTrack.hpp"
#include	"psJudge.h"

CBGFGTracker::CBGFGTracker()
{
	m_pKalmanProc = NULL;
	m_bInited	=FALSE;
	m_pMeasure	=	NULL;
	m_pControl	=	NULL;
	memset(m_warnTarget,	0x00,	sizeof(TRK_RECT_INFO)*SAMPLE_NUMBER);
	memset(m_warnTargetBK,	0x00,	sizeof(TRK_RECT_INFO)*SAMPLE_NUMBER);

	m_thredParam.searchThed	= 0.1;//0.2;
	m_thredParam.trkThred = 0.1;//0.2;
	m_thredParam.dispFrames = 25;
	m_thredParam.totalFrames = 100;
	m_thredParam.targetSize	= 225;
	m_thredParam.distRoi = 2.0;

	lostTargetBK.clear();
}

CBGFGTracker::~CBGFGTracker()
{
	DeInitCBFG();
}

void	CBGFGTracker::InitCBFG(	int x0, int y0, double DeltaT,int DP, int MP, int CP)
{
	if (m_bInited){
		DeInitCBFG();
	}

	if (m_pKalmanProc == NULL){
		m_pKalmanProc = new CKalman_mv();
		if ( m_pKalmanProc == NULL){
			m_bInited = FALSE;
			return;
		}
	}
	m_pKalmanProc->KalmanOpen(DP, MP, CP);
	if (!m_pKalmanProc->m_bInited){
		m_bInited = FALSE;
		return;
	}
	m_pKalmanProc->KalmanInitParam(x0, y0, DeltaT);

	if (m_pMeasure == NULL){
		m_pMeasure = new double[MP * 1];
		memset(m_pMeasure, 0, sizeof(*m_pMeasure));
	}
	if (m_pControl == NULL && CP > 0){
		m_pControl = new double[CP * 1];
		memset(m_pControl, 0, sizeof(*m_pControl));
	}

	m_bInited = TRUE;
}

void	CBGFGTracker::DeInitCBFG()
{
	if (m_pKalmanProc != NULL)
	{
		delete m_pKalmanProc;
		m_pKalmanProc = NULL;
	}
	if (m_pMeasure != NULL)
	{
		delete m_pMeasure;
		m_pMeasure = NULL;
	}
	if (m_pControl != NULL)
	{
		delete m_pControl;
		m_pControl = NULL;
	}
	m_bInited = FALSE;
}

void CBGFGTracker::KalmanPredict(int xout, int yout)
{
	/* Kalman滤波：更新yk，进行滤波，得到新的xk估计（k时刻）*/
	m_pMeasure[0] = (double)xout;
	m_pMeasure[1] = (double)yout;
	Kalman(m_pMeasure,	NULL);/* 进行Kalman滤波*/
}

void CBGFGTracker::Kalman(double *measure, double *control)
{
	m_pKalmanProc->KalmanPredict(control);
	m_pKalmanProc->KalmanCorrect(measure);
}

void	CBGFGTracker::SetTrkThred(TRK_THRED	 trkThred)
{
	memcpy(&m_thredParam, &trkThred, sizeof(TRK_THRED));
}

void	CBGFGTracker::SetTrkTarget(const std::vector<TRK_RECT_INFO>	warnTarget)
{
	int	i,	j,	k;
	int		nTargetNum	=	warnTarget.size();
	TRK_RECT_INFO	*pTrkInfo,	tgtInfo;
	cv::Rect	tgtRect,	trkRect;
	float	foverlap;
	bool	bContinue;

	i = 0;
	for(k=0; k<nTargetNum; k++)
	{
		tgtInfo	=	warnTarget[k];
		tgtRect	= tgtInfo.targetRect;
		if(tgtRect.width*tgtRect.height <m_thredParam.targetSize)//目标太小，不要跟踪
			continue;
		bContinue = false;
		for(j=0;j<SAMPLE_NUMBER;j++)
		{
			pTrkInfo = &m_warnTarget[j];
			if(pTrkInfo->trkState	== TRK_STATE_TRACK)
			{
				trkRect	=	pTrkInfo->targetRect;
				foverlap	= _bbOverlap(tgtRect, trkRect);
				if(foverlap > m_thredParam.searchThed/*0.2*/)//找到正在跟踪的重合目标，不需添加新目标
				{
					bContinue = true;
				}
			}
		}
		if(bContinue)
			continue;
		for(;i<SAMPLE_NUMBER;i++)
		{
			pTrkInfo = &m_warnTarget[i];
			if(pTrkInfo->trkState == TRK_STATE_IDLE)
			{
				*pTrkInfo	= tgtInfo;//赋值
				pTrkInfo->trkState = TRK_STATE_ACQ;
				break;
			}
		}
		if(i == SAMPLE_NUMBER)
			break;
	}
}

#if 0
static	void	_trackprocess(Pattern  *curPatterns,	 int	numPatterns,	TRK_RECT_INFO	*pTrkInfo, float trkThred)
{
	int	k;
	cv::Rect	tgtRect,	trkRect;
	Pattern	*pPat, *pBakPat;
	float	foverlap,	maxoverlap = 0.0;
	trkRect= pTrkInfo->targetRect;

	for(k=0; k<numPatterns;	k++)
	{
		pPat = &curPatterns[k];
		if(!pPat->bValid)
			continue;
		tgtRect	=	cv::Rect(pPat->lefttop.x,	pPat->lefttop.y, pPat->rightbottom.x-pPat->lefttop.x,	pPat->rightbottom.y-pPat->lefttop.y);
		foverlap	= _bbOverlap(tgtRect, trkRect);
		if(foverlap > maxoverlap)
		{
			maxoverlap	= foverlap;
			pBakPat = pPat;
		}
	}
	if(maxoverlap	> trkThred/*0.2*/)//表示跟踪到了目标
	{
		pBakPat->bValid	= false;
		pTrkInfo->targetRect	= cv::Rect(pBakPat->lefttop.x,	pBakPat->lefttop.y, pBakPat->rightbottom.x-pBakPat->lefttop.x,	pBakPat->rightbottom.y-pBakPat->lefttop.y);
		pTrkInfo->trk_frames++;
		pTrkInfo->lost_frames = 0;
	}
	else//没有跟踪到目标
	{
		pTrkInfo->trk_frames++;//等待处理
		pTrkInfo->lost_frames++;
		if(pTrkInfo->lost_frames > 8){//连续丢失20帧以上，进入闲置状态
			pTrkInfo->trkState	= TRK_STATE_IDLE;
		}
	}
}
#else
static	void	_trackprocess(const cv::Size sz, Pattern  *curPatterns,	 int	numPatterns,	int Idx, TRK_RECT_INFO	*pTrkInfo, float trkThred)
{
	int	k,idx, idx0;
	cv::Rect	tgtRect,	trkRect;
	Pattern	*pPat, *pBakPat;
	cv::Point2f centpt0, centpt1;
	float	foverlap,	maxoverlap = 0.0;
	trkRect= pTrkInfo->targetRect;

	for(k=0; k<numPatterns;	k++)
	{
		pPat = &curPatterns[k];
		if(!pPat->bValid)
			continue;
		tgtRect	=	cv::Rect(pPat->lefttop.x,	pPat->lefttop.y, pPat->rightbottom.x-pPat->lefttop.x,	pPat->rightbottom.y-pPat->lefttop.y);
		foverlap	= _bbOverlap(tgtRect, trkRect);
		if(foverlap > maxoverlap)
		{
			maxoverlap	= foverlap;
			pBakPat = pPat;
		}
	}
	if(maxoverlap	> trkThred/*0.2*/)//表示跟踪到了目标
	{
//		pBakPat->bValid	= false;
//		pTrkInfo->targetRect	= cv::Rect(pBakPat->lefttop.x,	pBakPat->lefttop.y, pBakPat->rightbottom.x-pBakPat->lefttop.x,	pBakPat->rightbottom.y-pBakPat->lefttop.y);

		pBakPat->IdxVec.push_back(Idx);
		pBakPat->lapVec.push_back(maxoverlap);

		pTrkInfo->trk_frames++;
		pTrkInfo->lost_frames = 0;
	}
	else//没有跟踪到目标
	{
		/*
			centpt0.x = pTrkInfo->targetRect.x + pTrkInfo->targetRect.width/2;
			centpt0.y = pTrkInfo->targetRect.y + pTrkInfo->targetRect.height/2;
			centpt1.x = pTrkInfo->targetVector[0].x + pTrkInfo->targetVector[0].width/2;
			centpt1.y = pTrkInfo->targetVector[0].y + pTrkInfo->targetVector[0].height/2;

			pTrkInfo->lostVel.x = (centpt0.x-centpt1.x)/500;
			pTrkInfo->lostVel.y = (centpt0.y-centpt1.y)/500;
			
			pTrkInfo->targetRect.x += pTrkInfo->lostVel.x;
			pTrkInfo->targetRect.y += pTrkInfo->lostVel.y;

			if(pTrkInfo->targetRect.x<0) pTrkInfo->targetRect.x=0;
			if(pTrkInfo->targetRect.y<0) pTrkInfo->targetRect.y=0;
			if(pTrkInfo->targetRect.x+pTrkInfo->targetRect.width>sz.width)
				pTrkInfo->targetRect.x = sz.width - pTrkInfo->targetRect.width;
			if(pTrkInfo->targetRect.y+pTrkInfo->targetRect.height>sz.height)
				pTrkInfo->targetRect.y = sz.height - pTrkInfo->targetRect.height;
		*/
		pTrkInfo->trk_frames++;//等待处理
		pTrkInfo->lost_frames++;
		if(pTrkInfo->lost_frames > 16){//连续丢失20帧以上，进入闲置状态
			pTrkInfo->trkState	= TRK_STATE_IDLE;
		}
	}
}
#endif

void	CBGFGTracker::TrackProcess(const cv::Size sz, Pattern  *curPatterns,	 int	numPatterns)
{
	int	i,	k;
	TRK_RECT_INFO	*pTrkInfo;
	CV_Assert(numPatterns<=SAMPLE_NUMBER);
	for(i=0;i<numPatterns;	i++)
	{
		curPatterns[i].bValid	= true;
		curPatterns[i].IdxVec.clear();
		curPatterns[i].lapVec.clear();
	}
	for(i=0;	i<SAMPLE_NUMBER;	i++)
	{
		pTrkInfo	= &m_warnTarget[i];
		if(pTrkInfo->trkState	==TRK_STATE_ACQ )
		{
			pTrkInfo->trkState	= TRK_STATE_TRACK;
			pTrkInfo->disp_frames	= 0;
			pTrkInfo->trk_frames = 1;
			pTrkInfo->lost_frames = 0;
			memcpy(&pTrkInfo->targetVector[0],&pTrkInfo->targetRect,sizeof(cv::Rect));
		}
		else	if (pTrkInfo->trkState	==TRK_STATE_TRACK)
		{
			_trackprocess(sz, curPatterns,	numPatterns,	i, pTrkInfo, m_thredParam.trkThred);
		}
		else if(pTrkInfo->trkState	==TRK_STATE_IDLE)
		{
			;
		}
	}

	for(i=0; i<numPatterns; i++){
		Pattern	*pPat = &curPatterns[i];
		int nsize = pPat->IdxVec.size() ;
		float maxlap=0.f;
		int maxIdx = 0;
		if(nsize>1){
			for(k=0; k<nsize; k++){
				if(pPat->lapVec[k] > maxlap){
					maxlap = pPat->lapVec[k];
					maxIdx = pPat->IdxVec[k];
				}
			}
			for(k=0; k<nsize; k++){
				if(pPat->IdxVec[k] == maxIdx){
					pTrkInfo	= &m_warnTarget[maxIdx];
					pTrkInfo->targetRect	= cv::Rect(pPat->lefttop.x,	pPat->lefttop.y, pPat->rightbottom.x-pPat->lefttop.x,	pPat->rightbottom.y-pPat->lefttop.y);

				
					if(pTrkInfo->trk_frames < 10)
						memcpy(&pTrkInfo->targetVector[pTrkInfo->trk_frames-1],&pTrkInfo->targetRect,sizeof(cv::Rect));
					else
					{
						memcpy(pTrkInfo->targetVector,&pTrkInfo->targetVector[1],9*sizeof(cv::Rect));
						memcpy(&pTrkInfo->targetVector[9],&pTrkInfo->targetRect,sizeof(cv::Rect));
					}
				#if 0
					printf("\n-------------------------------------------------------------------\n");
					for(int tmp = 0 ;tmp < 10;tmp++)
					{
						printf("LINE :%d   pTrkInfo->targetVector[%d] :(%d,%d,%d,%d)\n",__LINE__,tmp,pTrkInfo->targetVector[tmp].x,
							pTrkInfo->targetVector[tmp].y,pTrkInfo->targetVector[tmp].width,
							pTrkInfo->targetVector[tmp].height);
					}
					printf("\n-------------------------------------------------------------------\n\n");
				#endif
				
				}else{
					pTrkInfo	= &m_warnTarget[pPat->IdxVec[k]];
					pTrkInfo->lost_frames++;
				}
			}
		}else if(nsize == 1){
			pTrkInfo	= &m_warnTarget[pPat->IdxVec[0]];
			pTrkInfo->targetRect	= cv::Rect(pPat->lefttop.x,	pPat->lefttop.y, pPat->rightbottom.x-pPat->lefttop.x,	pPat->rightbottom.y-pPat->lefttop.y);		
			
				if(pTrkInfo->trk_frames < 10)
					memcpy(&pTrkInfo->targetVector[pTrkInfo->trk_frames-1],&pTrkInfo->targetRect,sizeof(cv::Rect));
				else
				{
					memcpy(pTrkInfo->targetVector,&pTrkInfo->targetVector[1],9*sizeof(cv::Rect));
					memcpy(&pTrkInfo->targetVector[9],&pTrkInfo->targetRect,sizeof(cv::Rect));
				}
				
			#if 0
				printf("\n-------------------------------------------------------------------\n");
				for(int tmp = 0 ;tmp < 10;tmp++)
				{
					printf("LINE :%d   pTrkInfo->targetVector[%d] :(%d,%d,%d,%d)\n",__LINE__,tmp,pTrkInfo->targetVector[tmp].x,
						pTrkInfo->targetVector[tmp].y,pTrkInfo->targetVector[tmp].width,
						pTrkInfo->targetVector[tmp].height);
				}
				printf("\n-------------------------------------------------------------------\n\n");
			#endif

		}
	}
}

void	CBGFGTracker::ClearTrkTarget(int	Idx)
{
	assert(Idx<SAMPLE_NUMBER);
	TRK_RECT_INFO	*pTrkInfo  = &m_warnTarget[Idx];
	pTrkInfo->trkState	= TRK_STATE_IDLE;
	pTrkInfo->warnType	= WARN_STATE_IDLE;
	pTrkInfo->trk_frames = 0;
	pTrkInfo->disp_frames	= 0;
	pTrkInfo->lost_frames = 0;
	
	memset(pTrkInfo->targetVector,0,10*sizeof(cv::Rect));
}

int	CBGFGTracker::TrackAnalyse(std::vector<cv::Point2i>		warnRoi)
{
	int	i,	k;
	int	dispFrames, totalFrames;
	float distRoi;
	cv::Scalar	color;
	std::vector<cv::Point2f>		polyRoi;
	cv::Point2f	rc_center;
	TRK_RECT_INFO	*pTrkInfo;
	int	warnState = WARN_STATE_IDLE;

	int	nsize =	warnRoi.size();
	CV_Assert(nsize	>2);
	polyRoi.resize(nsize);
	for(i=0; i<nsize; i++){
		polyRoi[i]	= cv::Point2f((float)warnRoi[i].x, (float)warnRoi[i].y);
	}
	dispFrames = m_thredParam.dispFrames;
	totalFrames = m_thredParam.totalFrames;
	distRoi	= m_thredParam.distRoi;

	for(k=0; k<SAMPLE_NUMBER;	k++){
		pTrkInfo	= &m_warnTarget[k];
		color	= cv::Scalar(0x00,0x00,0xFF);//red color
		if(pTrkInfo->trkState	== TRK_STATE_TRACK){

			int	nTrkFrames	= pTrkInfo->trk_frames;
			cv::Rect	targetRec	= pTrkInfo->targetRect;
			rc_center	= cv::Point2f((float)(targetRec.x+targetRec.width/2), (float)(targetRec.y+targetRec.height/2));

			double	distance	= cv::pointPolygonTest( polyRoi, rc_center, true );
			double	tgw	= targetRec.width;
			double	tgh	=  targetRec.height;
			double	diagd	 = sqrt(tgw*tgw+tgh*tgh);
			if(pTrkInfo->targetType	== TARGET_IN_POLYGON){
				if(distance<0.0	&& pTrkInfo->disp_frames == 0){//丢失
					pTrkInfo->warnType	=	WARN_STATE_LOST;
					pTrkInfo->disp_frames	=  pTrkInfo->disp_frames == 0?dispFrames:pTrkInfo->disp_frames;
					warnState	|=	WARN_STATE_LOST;
				}
			}else if(pTrkInfo->targetType	== TARGET_OUT_POLYGON){
				if(distance>0.0 && pTrkInfo->disp_frames == 0){//入侵
					pTrkInfo->warnType	=	WARN_STATE_INVADE;
					pTrkInfo->disp_frames	=  pTrkInfo->disp_frames == 0?dispFrames:pTrkInfo->disp_frames;
					warnState	|=	WARN_STATE_INVADE;
				}
			}else if(pTrkInfo->targetType	== TARGET_IN_EDGE){
				if(pTrkInfo->distance>0.0 && distance<0.0 && pTrkInfo->disp_frames == 0){//丢失
					pTrkInfo->warnType	=	WARN_STATE_LOST;
					pTrkInfo->disp_frames	=  pTrkInfo->disp_frames == 0?dispFrames:pTrkInfo->disp_frames;
					warnState	|=	WARN_STATE_LOST;
				}else	if(pTrkInfo->distance<0.0 && distance>0.0 && pTrkInfo->disp_frames == 0){//入侵
					pTrkInfo->warnType	=	WARN_STATE_INVADE;
					pTrkInfo->disp_frames	=  pTrkInfo->disp_frames == 0?dispFrames:pTrkInfo->disp_frames;	
					warnState	|=	WARN_STATE_INVADE;
				}
			}
			if(pTrkInfo->disp_frames >	0){
				pTrkInfo->disp_frames--;
				if(pTrkInfo->disp_frames	== 0)
					ClearTrkTarget(k);
			}
			if(nTrkFrames	> totalFrames){//清除跟踪
				ClearTrkTarget(k);
			}
			if(fabs(distance) >diagd*distRoi){//离警戒区超2倍对角线
				ClearTrkTarget(k);
			}
		}
	}
	return warnState;
}

bool chargeRatio(cv::Rect rect)
{
	double d;
	d = (double)rect.width/(double)rect.height ;
	if( d > 0.5  && d < 2.0 )
		return true;
	else
		return false;
}

bool CBGFGTracker::judgeEdgeInOut(TRK_RECT_INFO* curInfo )
{
	double	tgw	= curInfo->targetRect.width;
	double	tgh	= curInfo->targetRect.height;
	double	diagd	 = sqrt(tgw*tgw+tgh*tgh);
	double	maxd	=  diagd*3/4;
	double	mind	=	tgw>tgh?tgw/4:tgh/4;
	maxd = maxd<60.0?60.0:maxd;
	bool retFlag = false;
	if(curInfo->distance>=mind /*&& distance<=maxd	*/){
		curInfo->targetType = TARGET_IN_POLYGON;
		retFlag = true;
	}else if(curInfo->distance>-mind	&&	curInfo->distance<mind	){
		curInfo->targetType = TARGET_IN_EDGE;
		retFlag = true;
	}else if(/*distance>= -maxd	&&	*/curInfo->distance<=	-mind	){
		curInfo->targetType = TARGET_OUT_POLYGON;
		retFlag = false;
	}else{
		curInfo->targetType = TARGET_NORAM;
		retFlag = true;
	}
	return retFlag;	
}

void	CBGFGTracker::GetTrackTarget(std::vector<TRK_RECT_INFO> &lostTarget, std::vector<TRK_RECT_INFO> &invadeTarget, std::vector<TRK_RECT_INFO> &warnTarget , int frameIndex)
{
	lostTarget.clear();
	invadeTarget.clear();
	warnTarget.clear();



	int	k;
	TRK_RECT_INFO	*pTrkInfo;
	int i,j;
	for(k=0; k<SAMPLE_NUMBER;	k++)
	{	
		j=0;
		pTrkInfo	= &m_warnTarget[k];
		pTrkInfo->index = k;
		if(pTrkInfo->trkState	== TRK_STATE_TRACK)
		{
			if(pTrkInfo->warnType	==	WARN_STATE_LOST)
			{
				lostTarget.push_back(*pTrkInfo);
			}
			else if(pTrkInfo->warnType	==	WARN_STATE_INVADE)
			{
				invadeTarget.push_back(*pTrkInfo);
			}
			int chooseNumber ;
			if(frameIndex > HOLDING_NUM) 
				chooseNumber = 80;
			else
				chooseNumber = 30 ;
			if( pTrkInfo->trk_frames > chooseNumber )
				if(  chargeRatio(pTrkInfo->targetRect ))
					if(judgeEdgeInOut(pTrkInfo))
						warnTarget.push_back(*pTrkInfo);
		}
	}
}

static void _drawWarn(cv::Mat frame, TRK_RECT_INFO *warnTarget, bool bshow)
{
	int	k;
	cv::Scalar	color;
	TRK_RECT_INFO	*pTrkInfo;
	unsigned char fcolor = bshow?0xFF:0x00;
	for(k=0; k<SAMPLE_NUMBER;	k++)
	{
		pTrkInfo	= &warnTarget[k];
		color	= cv::Scalar(0x00,0x00,0xFF, fcolor);//red color
		if(pTrkInfo->trkState	== TRK_STATE_TRACK)
		{
			cv::Rect	targetRec	= pTrkInfo->targetRect;
			if(pTrkInfo->warnType	==	WARN_STATE_LOST)
				color	= cv::Scalar(0xFF,0x00,0x00,fcolor);//blue color
			else	if(pTrkInfo->warnType	==	WARN_STATE_INVADE)
				color	= cv::Scalar(0x00,0xFF,0x00,fcolor);//green color
			
			cv::Rect result = targetRec;
			char strDisplay[128];
			rectangle( frame, cv::Point( result.x, result.y ), cv::Point( result.x+result.width, result.y+result.height), color, 4, 8 );
			sprintf(strDisplay, "%d ", k) ;
			putText(frame, strDisplay,cv::Point( result.x+4, result.y+4 ),1,2,cv::Scalar(25,200,25,fcolor),2);
		}
	}
}


static void _drawWarnLost(cv::Mat frame, std::vector<LOST_RECT_INFO> &lostTarget ,bool bshow)
{
	int	k;
	cv::Scalar	color, color1;
	unsigned char fcolor = bshow?0xFF:0x00;
	std::vector<int> idxVector;
	idxVector.clear();
	#if 0
	if(bshow == false)
	{
		printf("\n*****************size = %d************************\n",lostTarget.size());
		std::vector<LOST_RECT_INFO>::iterator itr;
		for(itr = lostTarget.begin();itr!=lostTarget.end();++itr)
		{
			printf("lostTarget vector   Rect:(%d,%d,%d,%d) \n",(*itr).targetRect.x,(*itr).targetRect.y,
				(*itr).targetRect.width,(*itr).targetRect.height);
		}
		printf("\n**********************************************************\n");
	}
	#endif
	color	= cv::Scalar(0xA0,0xA0,0xA0, fcolor);
	color1	= cv::Scalar(0xA0,0xA0,0xA0, 0x00);
	for(k=0;k<lostTarget.size();k++)
	{
		cv::Rect result = lostTarget[k].targetRect;
		rectangle( frame, cv::Point( result.x, result.y ), cv::Point( result.x+result.width, result.y+result.height), color, 4, 8 );
		if(bshow == true)
		{
			lostTarget[k].disp_frames--;
			//printf("%s LINE:%d      Rect :(%d,%d,%d,%d)\n",__func__,__LINE__,result.x,result.y,result.width,result.height);
			if(lostTarget[k].disp_frames <= 0){
				idxVector.push_back(k);
				rectangle( frame, cv::Point( result.x, result.y ), cv::Point( result.x+result.width, result.y+result.height), color1, 4, 8 );
			}
		}
	}
	for(k=0; k<idxVector.size(); k++){
		int index = idxVector[k];
		lostTarget.erase(lostTarget.begin()+index);
	}
}

void	CBGFGTracker::DrawWarnTarget(cv::Mat	frame)
{
	_drawWarn(frame, m_warnTargetBK, false);
	_drawWarn(frame, m_warnTarget, true);
	memcpy(m_warnTargetBK, m_warnTarget, sizeof(TRK_RECT_INFO)*SAMPLE_NUMBER);
}

void	CBGFGTracker::DrawLostTarget(cv::Mat	frame,std::vector<LOST_RECT_INFO> &lostTarget)
{
	_drawWarnLost(frame, lostTargetBK, false);

	_drawWarnLost(frame, lostTarget, true);
	
	lostTargetBK.assign(lostTarget.begin(),lostTarget.end());	
}
