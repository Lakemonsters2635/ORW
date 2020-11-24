#pragma once

//#include <pcl/sample_consensus/sac_model.h>

class CRANSAC
{
public:
	CRANSAC();

	void RenderUI();
	void ShowRadiusLimits();
	void ShowAxis();
	void ShowEpsilon();
	void ShowConeAngle();

	void RegisterSettings();
	void ImportSettings();
	void Reset();


protected:
	int	m_nModel = -1;
	int m_nIterations = 1000;
	float m_fDistanceThreshhold = 0;
	float m_fRadiusLimits[2];
	float m_fAxis[3];
	float m_fEpsilon = 0;
	float m_fConeAngle[2];

};

