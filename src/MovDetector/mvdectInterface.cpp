#include "mvdectInterface.hpp"
#include "MovDetector.hpp"


CMvDectInterface *MvDetector_Create()
{
	CMvDectInterface	*pMvObj = NULL;
	CMoveDetector *pTmpMV = NULL;

	pTmpMV= (CMoveDetector*)new CMoveDetector;
	pMvObj = (CMvDectInterface*)pTmpMV;
	CV_Assert(pMvObj != NULL);
	
	pTmpMV->creat();
	return (CMvDectInterface*)pMvObj;
}

void MvDetector_Destory(CMvDectInterface *obj)
{
	CMoveDetector	*pMvObj = (CMoveDetector*)obj;
	if(pMvObj != NULL){
		delete pMvObj;
		obj = NULL;
	}
}
