#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include "psJudge.h"
#include "postDetector.hpp"

CPostDetect::CPostDetect():m_pPatterns (NULL),m_ptemp(NULL),m_pBitData (NULL)
{
	m_dwWidth = 0;
	m_dwHeight = 0;
	m_patternnum = 0;

	m_warnRoi.clear();
	
	m_warnTargetRec.clear();
	m_movTargetRec.clear();
	warnTargetBK.clear();
	m_edgeTargetRec.clear();
	m_warnState	=  WARN_STATE_IDLE;

	debugLostTarget.clear();
}

CPostDetect::~CPostDetect()
{
	DestroyMD();
	m_warnRoi.clear();
}

void CPostDetect::DestroyMD()
{
	if (m_pPatterns != NULL)
	{
		for(int i=0; i<SAMPLE_NUMBER; i++){
			m_pPatterns[i].IdxVec.clear();
			m_pPatterns[i].lapVec.clear();
		}
		delete []m_pPatterns;
		m_pPatterns = NULL;
	}
	if (m_ptemp != NULL)
	{
		delete []m_ptemp;
		m_ptemp = NULL;
	}
	if(m_pBitData != NULL)
		delete [] m_pBitData;
	m_pBitData = NULL;
	m_dwWidth = 0;
	m_dwHeight = 0;

	m_warnTargetRec.clear();
	m_movTargetRec.clear();
	warnTargetBK.clear();
	m_edgeTargetRec.clear();
}

BOOL  CPostDetect::InitializedMD(int lWidth, int lHeight, int lStride)
{
	if (m_dwHeight != lHeight || m_dwWidth != lWidth)
		DestroyMD();

	if (m_pPatterns == NULL)
	{
		m_pPatterns = new Pattern[SAMPLE_NUMBER];
		if (m_pPatterns == NULL)
		{
			return FALSE;
		}
		memset(m_pPatterns, 0, sizeof(Pattern)*SAMPLE_NUMBER);
	}
	if (m_ptemp == NULL)
	{
		m_ptemp = new TYPE_T[lWidth*lHeight];
		if (m_ptemp == NULL)
		{
			return FALSE;
		}
	}
	else
	{
		if (m_dwHeight != lHeight || m_dwWidth != lWidth && m_ptemp != NULL)
		{
			delete []m_ptemp;
			m_ptemp = new TYPE_T[lWidth*lHeight];
			if (m_ptemp == NULL)
			{
				return FALSE;
			}
		}
	}

	if(m_pBitData == NULL){
		m_pBitData = new BYTE [lStride * lHeight];
	}

	m_dwWidth = lWidth;
	m_dwHeight = lHeight;	
	return TRUE;
}

/*
GetMoveDetect(LPBYTE lpBitData,int lWidth, int lHeight, int iStride, int iscatter  = 5)
lpBitData : 标示图像数据指针
iscatter: 去掉总像素数小于该值得离散区域
m_patternnum ：最终得到的连通区域个数
m_pPattern[]:每个连通区域的位置
 */

#define BOL		4
BOOL  CPostDetect::GetMoveDetect(LPBYTE lpBitData,int lWidth, int lHeight, int iStride, int minArea,int maxArea, int iscatter/* = 20*/)
{
	BOOL iRet = TRUE;
	iRet = InitializedMD(lWidth, lHeight, iStride);
	if (!iRet) return FALSE;

	Pattern ptn[SAMPLE_NUMBER];
	memset(&ptn, 0, sizeof(ptn));
	memset(m_ptemp, 0, lWidth*lHeight*sizeof(TYPE_T));//全部置成0
	memset(m_iCount,0,sizeof(int)*SAMPLE_NUMBER);
	memset(m_iRelative,0,sizeof(int)*SAMPLE_NUMBER);
	memset(m_list,0,sizeof(int)*SAMPLE_NUMBER);

	Mat origImage = Mat(lHeight,lWidth,CV_8UC1,lpBitData);
	Mat thresh ;
	origImage.copyTo(thresh);

	memset(thresh.data, 0, iStride*BOL);
	memset(thresh.data+iStride*(lHeight-BOL), 0, iStride*BOL);
	for(int iCur=0; iCur<lHeight; iCur++){
		memset(thresh.data+iCur*iStride, 0, BOL);
		memset(thresh.data+iCur*iStride + lWidth - BOL, 0, BOL);
	}

//	Mat thresh = Mat(lHeight,lWidth,CV_8UC1,lpBitData);
	
	vector< vector<Point> > contours;
	vector< cv::Rect > boundRect;
	findContours(thresh, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);

	boundRect.resize(contours.size());
	for (int i = 0; i < contours.size(); i++)
       boundRect[i] = boundingRect(contours[i]);

	int patternnum = 0;//(boundRect.size()< SAMPLE_NUMBER) ? boundRect.size():SAMPLE_NUMBER;
	for(int i = 0; i< contours.size();i++){
		if(boundRect[i].width*boundRect[i].height >= minArea && boundRect[i].width*boundRect[i].height <= maxArea){
			ptn[patternnum].lefttop.x = boundRect[i].x;
			ptn[patternnum].lefttop.y = boundRect[i].y;
			ptn[patternnum].rightbottom.x = boundRect[i].x + boundRect[i].width;
			ptn[patternnum].rightbottom.y = boundRect[i].y + boundRect[i].height;	
			patternnum++;
			if(patternnum >= SAMPLE_NUMBER)
				break;
		}
	}
	
#if 0
	m_patternnum = 0;
	memcpy(m_pPatterns, &ptn, sizeof(ptn));
	m_patternnum = patternnum;
#else
	MergeRect(ptn, patternnum);
#endif
	return iRet;
}

inline void mergeOverLap(cv::Rect rec1,cv::Rect rec2,cv::Rect &outRec)
{
	cv::Rect tmp;
	if(rec1.x < rec2.x){
		tmp.x = rec1.x;
	}else{
		tmp.x = rec2.x;
	}
	if(rec1.x+rec1.width<rec2.x+rec2.width){
		tmp.width = rec2.x+rec2.width - tmp.x;
	}else{
		tmp.width = rec1.x+rec1.width - tmp.x;
	}
	
	if(rec1.y < rec2.y){
		tmp.y = rec1.y;
	}else{
		tmp.y = rec2.y;
	}
	if(rec1.y+rec1.height<rec2.y+rec2.height){
		tmp.height = rec2.y+rec2.height - tmp.y;
	}else{
		tmp.height = rec1.y+rec1.height - tmp.y;
	}
	outRec = tmp;

}



void CPostDetect::MergeRect(Pattern	ptn[], int num)
{
	int	i,	j;
	cv::Rect	rc1,	rc2, roi;
	int	status;
	for(j=0; j<num;	j++){
		ptn[j].bValid = true;
		ptn[j].bEdge = false;
	}
	for(j=0; j<num;	j++){
		if(	!ptn[j].bValid	)
			continue;
		rc1 = cv::Rect(ptn[j].lefttop,ptn[j].rightbottom);
		for(i=j+1; i<num;	i++){
			if(	!ptn[i].bValid	)
				continue;
			rc2 = cv::Rect(ptn[i].lefttop,ptn[i].rightbottom);
			status = _bInRect(rc1, rc2, roi);
			if(status == 1){
				//ptn[j].bValid = false;
			}else if(status == 2){
				//ptn[i].bValid = false;
			}else if(status == 0){//overlap
				mergeOverLap(rc1,rc2,rc1);
				//ptn[i].bValid = false;
			}
		}
	}
	m_patternnum = 0;
	for(j=0; j<num;	j++){
		if( ptn[j].bValid ){
			memcpy(m_pPatterns+m_patternnum , ptn+j, sizeof(Pattern));
			m_patternnum++;
		}
	}
}

void CPostDetect::MergeDetectRegion(std::vector<TRK_RECT_INFO>		&MVTarget)
{
	int i, j, k,status;
	cv::Rect	rc1,	rc2, roi;
	int nsize = MVTarget.size();

	std::vector<bool> validVector;
	std::vector<TRK_RECT_INFO>		tmpMVTarget;

	validVector.resize(nsize);
	tmpMVTarget.resize(nsize);

	for(i=0; i<nsize; i++){
		validVector[i] = true;
		tmpMVTarget[i] = MVTarget[i];
	}
	for(j=0; j<nsize;	j++){
		if( !validVector[j] )
			continue;
		rc1 = tmpMVTarget[j].targetRect;
		for(i=j+1; i<nsize;	i++){
			if(	!validVector[i]	)
				continue;
			rc2 = tmpMVTarget[i].targetRect;
			status = _bInRect(rc1, rc2, roi);
			if(status == 1){
				//validVector[j] = false;
			}else if(status == 2){
				//validVector[i] = false;
			}else if(status == 0){//overlap
				mergeOverLap(rc1,rc2,rc1);
				//tmpMVTarget[j].targetRect = rc1;
				//validVector[i] = false;
			}
		}
	}
	MVTarget.clear();
	for(j=0; j<nsize;	j++){
		if( validVector[j] ){
			MVTarget.push_back( tmpMVTarget[j]);
		}
	}

}

BOOL  CPostDetect::VHDilation(LPBYTE lpBitData, int lWidth, int lHeight, int iStride)
{
	BOOL iRet = TRUE;

	int dilation_type;
	int	dilation_elem = 0;
	if( dilation_elem == 0 ){ dilation_type = MORPH_RECT; }
	else if( dilation_elem == 1 ){ dilation_type = MORPH_CROSS; }
	else if( dilation_elem == 2) { dilation_type = MORPH_ELLIPSE; }

	int	dilation_size = 2;
	Mat element = getStructuringElement( dilation_type,	Size( 2*dilation_size + 1, 2*dilation_size+1 ), Point( dilation_size, dilation_size ) );
	cv::Mat	src	= cv::Mat(lHeight,	lWidth, CV_8UC1, lpBitData,	iStride*sizeof(BYTE));
	cv::dilate(src,	src, element);

	return iRet;
}

void	CPostDetect::setWarningRoi(std::vector<cv::Point2i>	warnRoi)
{
	int	k,	npoint = warnRoi.size();
	CV_Assert(npoint>2);
	m_warnRoi.resize(npoint);
	for(k=0; k<npoint; k++){
		m_warnRoi[k] = warnRoi[k];
	}
}

void	CPostDetect::setTrkThred(TRK_THRED		trkThred)
{
	m_bgfgTrack.SetTrkThred(trkThred);
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

#define		PATTERN_RECT_JUDGE	\
	ASSERT(curPattern.lefttop.x <= curPattern.rightbottom.x);	\
	ASSERT(curPattern.lefttop.y <= curPattern.rightbottom.y);	\
	ASSERT(curPattern.rightbottom.x < m_dwWidth);	\
	ASSERT(curPattern.rightbottom.y < m_dwHeight);	\
	curPattern.lefttop.x = (int)((curPattern.lefttop.x - (m_dwWidth>>1))*nScalX) + center.x;	\
	curPattern.lefttop.y = (int)((curPattern.lefttop.y - (m_dwHeight>>1))*nScalY) + center.y;	\
	curPattern.rightbottom.x = (int)((curPattern.rightbottom.x - (m_dwWidth>>1))*nScalX) + center.x;	\
	curPattern.rightbottom.y = (int)((curPattern.rightbottom.y - (m_dwHeight>>1))*nScalY) + center.y;	\
	\
	ASSERT(curPattern.lefttop.x <= curPattern.rightbottom.x);		\
	ASSERT(curPattern.lefttop.y <= curPattern.rightbottom.y);		\
	ASSERT(curPattern.rightbottom.x < nWidth);	\
	ASSERT(curPattern.rightbottom.y < nHeight);	


void	CPostDetect::MovTargetDetect(float nScalX /*= 1*/, float nScalY /*= 1*/)
{
	m_movTargetRec.clear();

	Pattern curPattern;
	cv::Point center;
	int i,	recW, recH, nWidth, nHeight;
	cv::Scalar	color;
	cv::Rect	 result;

	nWidth	= (int)(m_dwWidth*nScalX);
	nHeight	= (int)(m_dwHeight*nScalY);
	center.x = nWidth >> 1;
	center.y = nHeight >> 1;

	//std::vector<cv::Point2f>		polyRoi;
	cv::Point2f rc_center;
	//int	nsize = m_warnRoi.size();
	//CV_Assert(nsize	>2);
	//polyRoi.resize(nsize);

	//for(i=0; i<nsize; i++){
	//	polyRoi[i]	= cv::Point2f((float)m_warnRoi[i].x, (float)m_warnRoi[i].y);
	//}

	for(int irec = 0; irec<m_patternnum; irec++){
		memcpy(&curPattern, &m_pPatterns[irec], sizeof(Pattern));

		PATTERN_RECT_JUDGE;

		int	recW	= curPattern.rightbottom.x	- curPattern.lefttop.x;
		int	recH	=	curPattern.rightbottom.y	- curPattern.lefttop.y;

		rc_center	= cv::Point2f((float)(curPattern.lefttop.x +curPattern.rightbottom.x)/2.0, (float)(curPattern.lefttop.y+curPattern.rightbottom.y)/2.0);
		cv::Rect rect((int)(rc_center.x-recW/2),	(int)(rc_center.y-recH/2),	 recW, recH);
		//double	distance	= cv::pointPolygonTest( polyRoi, rc_center, true );///1.0

//		if(distance >0.0)
		{
			TRK_RECT_INFO	curInfo;
			curInfo.targetRect		=	rect;
			curInfo.distance		= 0.0;
			curInfo.disp_frames	=	0;
			curInfo.warnType	=	WARN_STATE_IDLE;
			curInfo.trk_frames	= 0;
			curInfo.targetType = TARGET_IN_POLYGON;
			m_movTargetRec.push_back(curInfo);
		}
		//printf("m_movTargetRec size = %d \n",m_movTargetRec.size());
	}
}

void 	CPostDetect::validTarget(std::vector<TRK_RECT_INFO>	TmpMVTarget, std::vector<TRK_RECT_INFO>	&MVTarget)
{
	int i, nsize;
	std::vector<cv::Point2f>		polyRoi;

	nsize = m_warnRoi.size();
	CV_Assert(nsize	>2);
	polyRoi.resize(nsize);

	for(i=0; i<nsize; i++){
		polyRoi[i]	= cv::Point2f((float)m_warnRoi[i].x, (float)m_warnRoi[i].y);
	}

	MVTarget.clear();
	nsize = TmpMVTarget.size();
	for(i=0; i<nsize; i++){
		cv::Rect targetRec = TmpMVTarget[i].targetRect;
		cv::Point2f rc_center = cv::Point2f((float)targetRec.x+targetRec.width/2.0, (float)targetRec.y+targetRec.height/2.0);
		double	distance	= cv::pointPolygonTest( polyRoi, rc_center, true );///1.0
		if(distance >0.0){
			TRK_RECT_INFO	curInfo = TmpMVTarget[i];
			curInfo.distance = distance;
			MVTarget.push_back(curInfo);
		}
	}
}

void	CPostDetect::MovTargetDraw(cv::Mat	frame)
{
	DrawWarnTarget(frame,	m_movTargetRec);
}

void	CPostDetect::getMoveTarget(std::vector<TRK_RECT_INFO> &moveTarget)
{
	_copyTarget(m_movTargetRec, moveTarget);
}

void	CPostDetect::edgeTargetDetect(float nScalX /*= 1*/, float nScalY /*= 1*/)
{
	Pattern curPattern;
	cv::Point center;
	int i,	recW, recH, nWidth, nHeight;
	cv::Scalar	color;
	cv::Rect	 result;

	m_edgeTargetRec.clear();

	nWidth	= (int)(m_dwWidth*nScalX);
	nHeight	= (int)(m_dwHeight*nScalY);
	center.x = nWidth >> 1;
	center.y = nHeight >> 1;

	std::vector<cv::Point2f>		polyRoi;
	cv::Point2f	rc_center;
	int	nsize = m_warnRoi.size();
	CV_Assert(nsize	>2);
	polyRoi.resize(nsize);
	for(i=0; i<nsize; i++){
		polyRoi[i]	= cv::Point2f(m_warnRoi[i].x, m_warnRoi[i].y);
	}
	for(int irec = 0; irec<m_patternnum; irec++){
		memcpy(&curPattern, &m_pPatterns[irec], sizeof(Pattern));

		PATTERN_RECT_JUDGE;

		bool	bEdgeRect = false;
		cv::Scalar	colorNew;
		cv::Point ptStart , ptEnd;
		cv::Rect rect(curPattern.lefttop.x,curPattern.lefttop.y,curPattern.rightbottom.x-curPattern.lefttop.x,curPattern.rightbottom.y-curPattern.lefttop.y);
		for(i=0; i<nsize; i++){
			ptStart = polyRoi[i];
			ptEnd = polyRoi[(i+1)%nsize];
			bEdgeRect = IsLineIntersectRect(ptStart, ptEnd, rect);
			if(bEdgeRect)
				break;
		}
		if(bEdgeRect){
			TRK_RECT_INFO	curInfo;
			curInfo.targetRect		=	rect;
			curInfo.distance		= 0;
			curInfo.disp_frames	=	0;
			curInfo.warnType	=	WARN_STATE_EDGE;
			curInfo.trk_frames	= 0;
			curInfo.targetType = TARGET_IN_EDGE;
			m_edgeTargetRec.push_back(curInfo);
		}

	}
}

void	CPostDetect::edgeTargetDraw(cv::Mat	frame)
{
	DrawWarnTarget(frame,	m_edgeTargetRec);
}

void	CPostDetect::getEdgeTarget(std::vector<TRK_RECT_INFO> &edgeTarget)
{
	_copyTarget(m_edgeTargetRec, edgeTarget);
}

void	CPostDetect::warnTargetSelect( float nScalX /*= 1*/, float nScalY /*= 1*/)
{
	m_warnTargetRec.clear();

	Pattern curPattern;
	cv::Point center;
	int i,	recW, recH, nWidth, nHeight;
	cv::Scalar	color;
	cv::Rect	 result;

	nWidth	= (int)(m_dwWidth*nScalX);
	nHeight	= (int)(m_dwHeight*nScalY);
	center.x = nWidth >> 1;
	center.y = nHeight >> 1;

	std::vector<cv::Point2f>		polyRoi;
	cv::Point2f rc_center;
	int	nsize = m_warnRoi.size();
	CV_Assert(nsize	>2);
	polyRoi.resize(nsize);
	for(i=0; i<nsize; i++){
		polyRoi[i]	= cv::Point2f((float)m_warnRoi[i].x, (float)m_warnRoi[i].y);
	}
	for(int irec = 0; irec<m_patternnum; irec++){
		memcpy(&curPattern, &m_pPatterns[irec], sizeof(Pattern));

		PATTERN_RECT_JUDGE;

		int	recW	= curPattern.rightbottom.x	- curPattern.lefttop.x;
		int	recH	=	curPattern.rightbottom.y	- curPattern.lefttop.y;
		if(recW*recH >(nWidth*nHeight>>4)){
			continue;
		}
		rc_center	= cv::Point2f((float)(curPattern.lefttop.x +curPattern.rightbottom.x)/2.0, (float)(curPattern.lefttop.y+curPattern.rightbottom.y)/2.0);
		cv::Rect rect((int)(rc_center.x-recW/2),	(int)(rc_center.y-recH/2),	 recW, recH);
		double	distance	= cv::pointPolygonTest( polyRoi, rc_center, true );
		double	tgw	= recW;
		double	tgh	=  recH;
		double	diagd	 = sqrt(tgw*tgw+tgh*tgh);
		double	maxd	=  diagd*3/4;
		double	mind	=	tgw>tgh?tgw/4:tgh/4;
		maxd = maxd<60.0?60.0:maxd;

		TRK_RECT_INFO	curInfo;
		curInfo.targetRect		=	rect;
		curInfo.distance		= distance;
		curInfo.disp_frames	=	0;
		curInfo.warnType	=	WARN_STATE_IDLE;
		curInfo.trk_frames	= 0;

		if(distance>=mind && distance<=	maxd	){
			curInfo.targetType = TARGET_IN_POLYGON;
			m_warnTargetRec.push_back(curInfo);
		}else if(distance	> -mind	&&	distance<	mind	){
			curInfo.targetType = TARGET_IN_EDGE;
			m_warnTargetRec.push_back(curInfo);
		}else if(distance	>= -maxd	&&	distance<=	-mind	){
			curInfo.targetType = TARGET_OUT_POLYGON;
			m_warnTargetRec.push_back(curInfo);
		}
	}
//	DrawWarnTarget(src,	m_warnTargetRec);
}

void	CPostDetect::warnTargetSelect_New(const std::vector<TRK_RECT_INFO>	MVTarget)
{
	int i, nsize;
	
	std::vector<cv::Point2f>		polyRoi;
	nsize = m_warnRoi.size();
	CV_Assert(nsize >2);
	polyRoi.resize(nsize);

	for(i=0; i<nsize; i++){
		polyRoi[i]	= cv::Point2f((float)m_warnRoi[i].x, (float)m_warnRoi[i].y);
	}

	m_warnTargetRec.clear();
	nsize = MVTarget.size();
	
	if(nsize > SAMPLE_NUMBER)
		nsize = SAMPLE_NUMBER;

	for(i=0; i<nsize; i++){
		cv::Rect targetRec = MVTarget[i].targetRect;
		cv::Point2f rc_center = cv::Point2f((float)targetRec.x+targetRec.width/2.0, (float)targetRec.y+targetRec.height/2.0);
		//double	distance	= cv::pointPolygonTest( polyRoi, rc_center, true );///1.0

		TRK_RECT_INFO	curInfo;
		curInfo.targetRect		= targetRec;
		//curInfo.distance		= distance;
		curInfo.disp_frames	=	0;
		curInfo.warnType	=	WARN_STATE_IDLE;
		curInfo.trk_frames	= 0;

		m_warnTargetRec.push_back(curInfo);
	}
}

static void _drawWarnTarget(cv::Mat	frame,	std::vector<TRK_RECT_INFO>	warnTarget, bool bshow)
{
	int	k;
	int	nsize	=	warnTarget.size();
	TRK_RECT_INFO	curInfo;
	unsigned char fcolor = bshow?0xFF:0x00;
	cv::Scalar	color;
	for(k=0;	k<nsize;	k++){
		curInfo	=	warnTarget[k];
		if(curInfo.targetType	==	TARGET_IN_POLYGON){
			color	=	cv::Scalar(0xFF,	0x00,	0x00, fcolor);
		}else if (curInfo.targetType	==	TARGET_OUT_POLYGON){
			color	=	cv::Scalar(0x00,	0xFF,	0x00, fcolor);
		}else if (curInfo.targetType	==	TARGET_IN_EDGE){
			color	=	cv::Scalar(0x00,	0x00,	0xFF, fcolor);
		}
		cv::Rect result = curInfo.targetRect;
		rectangle( frame, Point( result.x, result.y ), Point( result.x+result.width, result.y+result.height), color, 4, 8 );
	}
}

void	CPostDetect::DrawWarnTarget(cv::Mat	frame,	std::vector<TRK_RECT_INFO>	warnTarget)
{
	_drawWarnTarget(frame, warnTargetBK, false);

	warnTargetBK.clear();
	int	k;
	int	nsize	=	warnTarget.size();
	TRK_RECT_INFO	curInfo;
	warnTargetBK.resize(nsize);
	for(k=0; k<nsize; k++){
		warnTargetBK[k] = warnTarget[k];
	}

	_drawWarnTarget(frame, warnTarget, true);
}

void	CPostDetect::SetTargetBGFGTrk()
{
	m_bgfgTrack.SetTrkTarget(m_warnTargetRec);
}

void	CPostDetect::WarnTargetBGFGTrk(const cv::Size sz)
{
	m_bgfgTrack.TrackProcess(sz, m_pPatterns,	m_patternnum);
}

void	CPostDetect::WarnTargetBGFGTrk_New(const cv::Size sz)
{
	int i, nsize = m_warnTargetRec.size();
	m_patternnum = nsize;
	for(i=0; i<nsize; i++){
		cv::Rect targetRec = m_warnTargetRec[i].targetRect;
		m_pPatterns[i].lefttop.x= targetRec.x;
		m_pPatterns[i].lefttop.y= targetRec.y;
		m_pPatterns[i].rightbottom.x= targetRec.x+targetRec.width;
		m_pPatterns[i].rightbottom.y= targetRec.y+targetRec.height;
	}
	m_bgfgTrack.TrackProcess(sz, m_pPatterns,	m_patternnum);
	m_bgfgTrack.DeleteOverlap();
}



void CPostDetect::WarnTargetValidAnalyse(std::vector<TRK_RECT_INFO> &warnTarget,vibeModel_Sequential_t *model,const uint8_t *image_data,
		float nScalX , float nScalY , cv::Size offsize)
{
	bool reflag ;
	int nsize = warnTarget.size();
	int recNum;	
	for(int i = 0; i < nsize ; i++ )
	{
		recNum = 0;
		reflag = false;
		assert(warnTarget[i].trkState == TRK_STATE_TRACK);

		int x =  warnTarget[i].targetVector[0].x;
		int y =  warnTarget[i].targetVector[0].y;
		int w =  warnTarget[i].targetVector[0].width;
		int h =  warnTarget[i].targetVector[0].height;

		//printf("%s  line: %d     x,y,w,h = (%d,%d,%d,%d)\n",__func__,__LINE__,x,y,w,h);
		
		if(warnTarget[i].trk_frames > 5)
		{
			for( int j = 1 ; j < warnTarget[i].trk_frames ;  ++j )
			{
				if( ( abs((x + w/2) - (warnTarget[i].targetVector[j].x + warnTarget[i].targetVector[j].width/2)) < 1)
					&& ( abs((y + h/2) - (warnTarget[i].targetVector[j].y + warnTarget[i].targetVector[j].height/2)) < 1)
					//&& ( abs( w - warnTarget[i].targetVector[j].width) < 40 ) 
					//&& ( abs( h - warnTarget[i].targetVector[j].height) < 40)
					)
				{
					recNum++;//reflag = true;
				}
				else
				{
					recNum = 0;//reflag = false;
					break;
				}
			}
		}
		else
			recNum = 0;//reflag = true;

		if(recNum >= 9)
			reflag = true;
		else
			reflag = false;

		if(reflag)
		{
			x = (x-offsize.width)/nScalX;
			y = (y-offsize.height)/nScalY;
			w /= nScalX;
			h /= nScalY;
			
			if(model != NULL)
			{
				libvibeModel_Sequential_Update_8u_C3R_part(model,image_data,x,y,w,h);

				LOST_RECT_INFO pTmp;
				memcpy(&pTmp.targetRect,&warnTarget[i].targetVector[0],sizeof(cv::Rect));
				pTmp.disp_frames = 15;
				debugLostTarget.push_back(pTmp);

				m_bgfgTrack.ClearTrkTarget(warnTarget[i].index);
				warnTarget.erase(warnTarget.begin() + i);
			}
		}
	}
}


void	CPostDetect::TargetBGFGAnalyse()
{
	m_warnState	=	m_bgfgTrack.TrackAnalyse(m_warnRoi);
}

void CPostDetect::GetMeanVar(const cv::Mat frame, std::vector<TRK_RECT_INFO> &warnTarget, float nScalX , float nScalY, cv::Size offsize)
{
	int k, nsize = warnTarget.size();
	TRK_RECT_INFO	*pTrkInfo;
	cv::Rect rec;
	cv::Scalar mean,var;
	for(k=0; k<nsize; k++){
		pTrkInfo = &warnTarget[k];
		rec = pTrkInfo->targetRect;
		rec.x =(rec.x-offsize.width)/nScalX;
		rec.y =(rec.y-offsize.height)/nScalY;
		rec.width /=nScalX;	rec.height /=nScalY;
		if(( rec.x  > 0 && (rec.x + rec.width < frame.cols))
			&& ( rec.y > 0 && (rec.y + rec.height < frame.rows)))
		{
			meanStdDev(frame(rec), mean, var);
			pTrkInfo->mean = mean.val[0];
			pTrkInfo->var = var.val[0];
		}
		else
		{
			pTrkInfo->mean = 0;
			pTrkInfo->var = 0;
		}
	}
}

void	CPostDetect::GetBGFGTarget(std::vector<TRK_RECT_INFO> &lostTarget, std::vector<TRK_RECT_INFO> &invadeTarget, std::vector<TRK_RECT_INFO> &warnTarget,int frameIndex)
{
	m_bgfgTrack.GetTrackTarget(lostTarget, invadeTarget, warnTarget , frameIndex, m_warnRoi);
}

void	CPostDetect::DrawBGFGTarget(cv::Mat	frame)
{
	m_bgfgTrack.DrawWarnTarget(frame);
}

void	CPostDetect::DrawLOSTTarget(cv::Mat	frame)
{
	m_bgfgTrack.DrawLostTarget(frame,debugLostTarget);
}

void	CPostDetect::initWarnTarget()
{
	m_bgfgTrack.initWarnTarget();
	return;
}

