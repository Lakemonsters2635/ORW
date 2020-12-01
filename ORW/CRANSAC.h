#pragma once

#include <pcl/sample_consensus/sac_model.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>

#include "PCLUtils.h"

#include <vector>
#include <iostream>

#define	N_COLORS	11
#define IM_COLOR(c)	{ c.r/255.0f, c.g/255.0f, c.b/255.0f, 1.0f }


struct CRANSAC_FEATURE
{
	CRANSAC_FEATURE(pcl_ptr p) { cloud = p; }
	pcl_ptr	cloud;
	pcl::SacModel	m_Model;
	pcl_model_coefficients_ptr m_pCoefficients;
};

using feature_ptr = CRANSAC_FEATURE*;

//int operator<(const feature_ptr& p1, const feature_ptr& p2)
//{
//	return (p1->cloud->size() < p2->cloud->size());
//}

//struct Feature
//{
//	Feature(pcl_ptr p) { cloud = p; }
//	pcl_ptr	cloud;
//	pcl::ModelCoefficients::Ptr coefficients;
//};
//
//typedef Feature* feature_ptr;


class CRANSAC
{
public:
	CRANSAC();

	bool FindFeatures(const rs2::points& rs2_points);
	const std::vector<feature_ptr>& GetLayers() { return m_DisplayFeatures;  }
	bool IsBusy() { return m_bBusy; }
	bool Stop()
	{
		m_bStop = true;
		processing_thread->join();			// Wait for thread to exit.
	}

	void RenderUI();
	void ShowRadiusLimits();
	void ShowAxis();
	void ShowEpsilon();
	void ShowConeAngle();

	void ShowResultDistance(int nLayer);
	void ShowResultAngle(int nLayer);
	void ShowResultDistanceXZ(int nLayer);
	void ShowResultAvgDistance(int nLayer);

	void RegisterSettings();
	void ImportSettings();
	void Reset();


protected:
	int m_nMethod = pcl::SAC_RANSAC;
	int	m_nModel = -1;
	int m_nIterations = 1000;
	float m_fDistanceThreshhold = 0;
	float m_fRadiusLimits[2];
	float m_fAxis[3];
	float m_fEpsilon = 0;
	float m_fConeAngle[2];

	std::thread* processing_thread = nullptr;
	bool m_bBusy = false;
	bool m_bStop = false;
	Semaphore work_to_do;

	pcl_ptr pointCloud;
	bool usesNormals;

	std::vector<feature_ptr> m_FoundFeatures;

	std::mutex m_DisplayUpdateLock;
	std::vector<feature_ptr> m_DisplayFeatures;
};

