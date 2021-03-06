#ifndef		_BG_FG_TRACK_H_
#define		_BG_FG_TRACK_H_

#include	"Kalman.h"
#include	"infoHead.h"

using namespace mv_detect;

class CBGFGTracker
{
public:
	CBGFGTracker();
	virtual	~CBGFGTracker();

	void	InitCBFG(	int x0, int y0, double DeltaT,int DP, int MP, int CP);
	void	DeInitCBFG();

	void	Kalman(double *measure, double *control);
	void	KalmanPredict(int xout, int yout);

	void	SetTrkTarget(const std::vector<TRK_RECT_INFO>	warnTarget);
	void	TrackProcess(const cv::Size sz, Pattern  *curPatterns,	 int	numPatterns);
	int		TrackAnalyse(std::vector<cv::Point2i>	warnRoi);
	void	GetTrackTarget(std::vector<TRK_RECT_INFO> &lostTarget, std::vector<TRK_RECT_INFO> &invadeTarget, std::vector<TRK_RECT_INFO> &warnTarget,int frameIndex,std::vector<cv::Point2i> warnRoi);
	void	ClearTrkTarget(int	Idx);

	void	SetTrkThred(TRK_THRED	 trkThred);
	void	DrawWarnTarget(cv::Mat	frame);
	void	DrawLostTarget(cv::Mat	frame,std::vector<LOST_RECT_INFO> &lostTarget);
	
	bool  	judgeEdgeInOut(TRK_RECT_INFO* curInfo, std::vector<cv::Point2i> warnRoi);

	void 	DeleteOverlap();
	void	initWarnTarget();
	
public:

	TRK_RECT_INFO		m_warnTarget[SAMPLE_NUMBER];
	TRK_RECT_INFO		m_warnTargetBK[SAMPLE_NUMBER];

	CKalman_mv* m_pKalmanProc;
	double*  m_pMeasure;
	double*  m_pControl;

	std::vector<LOST_RECT_INFO>  lostTargetBK;

protected:
	TRK_THRED		m_thredParam;

private:

	BOOL		m_bInited;
};

#endif
