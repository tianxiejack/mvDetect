#ifndef		_POST_DETECTOR_H_
#define		_POST_DETECTOR_H_

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "osa_sem.h"
#include "osa_tsk.h"

#include "infoHead.h"
#include "BGFGTrack.hpp"

#include "vibe-background-sequential.h"

using namespace cv;
using namespace std;
using namespace mv_detect;

//typedef    unsigned char TYPE_T ;
typedef    short  TYPE_T;

class CPostDetect 
{
public:
	CPostDetect();
	virtual ~CPostDetect();
	BOOL    	GetMoveDetect(LPBYTE lpBitData,int lWidth, int lHeight, int iStride,int minArea,int maxArea, int iscatter = 20);
	BOOL   	VHDilation(LPBYTE lpBitData, int lWidth, int lHeight, int iStride);
	BOOL   	InitializedMD(int lWidth, int lHeight, int lStride);
	void		DestroyMD();

	void 		MergeDetectRegion(std::vector<TRK_RECT_INFO>		&MVTarget);
	void		MergeRect(Pattern	ptn[], int num);
	void		setWarningRoi(std::vector<cv::Point2i>	warnRoi);
	void		setTrkThred(TRK_THRED		trkThred);

	void		edgeTargetDetect( float nScalX = 1, float nScalY = 1);
	void		edgeTargetDraw(cv::Mat	frame);
	void		getEdgeTarget(std::vector<TRK_RECT_INFO> &edgeTarget);

	void		MovTargetDetect(float nScalX = 1, float nScalY = 1);
	void		MovTargetDraw(cv::Mat	frame);
	void		getMoveTarget(std::vector<TRK_RECT_INFO> &moveTarget);

	void		warnTargetSelect(float nScalX = 1, float nScalY = 1);
	void		warnTargetSelect_New(const std::vector<TRK_RECT_INFO>	MVTarget);
	void		DrawWarnTarget(cv::Mat	frame,	std::vector<TRK_RECT_INFO>	warnTarget);

	void		SetTargetBGFGTrk();
	void		WarnTargetBGFGTrk(const cv::Size sz);
	void		WarnTargetBGFGTrk_New(const cv::Size sz);
	void		TargetBGFGAnalyse();
	void		GetBGFGTarget(std::vector<TRK_RECT_INFO> &lostTarget, std::vector<TRK_RECT_INFO> &invadeTarget, std::vector<TRK_RECT_INFO> &warnTarget , int frameIndex);
	void		DrawBGFGTarget(cv::Mat	frame);
	int			GetWarnState(){return m_warnState;};
	void 		validTarget(std::vector<TRK_RECT_INFO>	TmpMVTarget, std::vector<TRK_RECT_INFO>	&MVTarget);
	void		WarnTargetValidAnalyse(std::vector<TRK_RECT_INFO> &warnTarget,vibeModel_Sequential_t *model,const uint8_t *image_data,	\
			float nScalX , float nScalY, cv::Size offsize );

	void		DrawLOSTTarget(cv::Mat	frame);
	void 		GetMeanVar(const cv::Mat frame, std::vector<TRK_RECT_INFO> &warnTarget, float nScalX , float nScalY, cv::Size offsize);
	void 		initWarnTarget();
	
public:
	Pattern  *m_pPatterns;
	int			m_patternnum; //
	TYPE_T     *m_ptemp;
	BYTE     *m_pBitData;
	int      m_dwWidth;
	int      m_dwHeight;
	int      m_iCount[SAMPLE_NUMBER];
	int      m_iRelative[SAMPLE_NUMBER];
	int      m_list[SAMPLE_NUMBER];

	CBGFGTracker		m_bgfgTrack;

	std::vector<cv::Point2i>		m_warnRoi;
	std::vector<TRK_RECT_INFO>		m_warnTargetRec;
	std::vector<TRK_RECT_INFO>		m_movTargetRec;
	std::vector<TRK_RECT_INFO>		m_edgeTargetRec;
	int								m_warnState;
	std::vector<TRK_RECT_INFO>		warnTargetBK;

	std::vector<LOST_RECT_INFO>		debugLostTarget;
};


#endif
