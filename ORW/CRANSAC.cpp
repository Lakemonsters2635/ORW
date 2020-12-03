#include "all_includes.h"

#include "CRANSAC.h"

#include "PCLUtils.h"
#include "PointCloudDisplay.h"

struct MethodInfo
{
	const char* Name;
	int Method;

	static bool ItemGetter(void* data, int n, const char** out_str)
	{
		*out_str = ((MethodInfo*)data)[n].Name;
		return true;
	}
};

static MethodInfo Methods[] =
{ 
	{ "RANSAC", pcl::SAC_RANSAC },
	{ "LMEDS", pcl::SAC_LMEDS },
	{ "MSAC", pcl::SAC_MSAC },
	{ "RRANSAC", pcl::SAC_RRANSAC },
	{ "RMSAC", pcl::SAC_RMSAC },
	{ "MLESAC", pcl::SAC_MLESAC },
	{ "PROSAC", pcl::SAC_PROSAC }
};


struct ModelInfo
{
	const char* Name;
	pcl::SacModel	model;
	bool usesNormals;
	void (CRANSAC::* paramFunctions[3])();
	void (CRANSAC::* resultFunctions[3])(int);
	const char** coefLabels;

	static bool ItemGetter(void* data, int n, const char** out_str)
	{
		*out_str = ((ModelInfo*)data)[n].Name;
		return true;
	}
};
using cstr = const char*;

static cstr planeLabels[] = { "normal_x", "normal_y", "normal_z", "d", nullptr };
static cstr lineLabels[] = { "point_on_line_x", "point_on_line_y", "point_on_line_z", "line_direction_x", "line_direction_y", "line_direction_z", nullptr };
static cstr sphereLabels[] = { "center_x", "center_y", "center_z", "radius", nullptr };
static cstr circle2DLabels[] = { "center_x", "center_y", "radius", nullptr };
static cstr circle3DLabels[] = { "center_x", "center_y", "center_z", "radius", "normal_x", "normal_y", "normal_z", nullptr };
static cstr cylinderLabels[] = { "point_on_axis_x", "point_on_axis_y", "point_on_axis_z", "radius", "axis_direction_x", "axis_direction_y", "axis_direction_z", nullptr };
static cstr coneLabels[] = { "apex_x", "apex_y", "apex_z", "axis_direction_x", "axis_direction_y", "axis_direction_z", "opening_angle", nullptr };

static ModelInfo Models[] =
{
	{ "None", (pcl::SacModel) -1, false, { nullptr }, { nullptr } },
	{ "Plane", pcl::SACMODEL_PLANE, false, { nullptr }, { &CRANSAC::ShowResultAngle, &CRANSAC::ShowResultDistance, nullptr }, planeLabels },
	{ "Normal Plane", pcl::SACMODEL_NORMAL_PLANE, true, { nullptr }, { &CRANSAC::ShowResultAngle, &CRANSAC::ShowResultDistance, nullptr }, planeLabels },
	{ "Line", pcl::SACMODEL_LINE, false, { nullptr }, { nullptr }, lineLabels },
	{ "Circle2D", pcl::SACMODEL_CIRCLE2D, false, { &CRANSAC::ShowRadiusLimits, nullptr }, { &CRANSAC::ShowResultAvgDistance, nullptr }, circle2DLabels },
	{ "Circle3D", pcl::SACMODEL_CIRCLE3D, false, { &CRANSAC::ShowRadiusLimits, nullptr }, { &CRANSAC::ShowResultAngle, &CRANSAC::ShowResultDistanceXZ, nullptr }, circle3DLabels },
	{ "Sphere", pcl::SACMODEL_SPHERE, false, { &CRANSAC::ShowRadiusLimits, nullptr }, { &CRANSAC::ShowResultAngle, &CRANSAC::ShowResultDistanceXZ, nullptr }, sphereLabels },
	{ "Normal Sphere", pcl::SACMODEL_NORMAL_SPHERE, true, { &CRANSAC::ShowRadiusLimits, nullptr }, { &CRANSAC::ShowResultAngle, &CRANSAC::ShowResultDistanceXZ, nullptr }, sphereLabels },
	{ "Parallel Line", pcl::SACMODEL_PARALLEL_LINE, false, { &CRANSAC::ShowAxis, &CRANSAC::ShowEpsilon, nullptr }, { nullptr }, lineLabels },
	{ "Parallel Plane", pcl::SACMODEL_PARALLEL_PLANE, false, { &CRANSAC::ShowAxis, &CRANSAC::ShowEpsilon, nullptr }, { &CRANSAC::ShowResultAngle, &CRANSAC::ShowResultDistance, nullptr }, planeLabels },
	{ "Normal Parallel Plane", pcl::SACMODEL_NORMAL_PARALLEL_PLANE, true, { &CRANSAC::ShowAxis, &CRANSAC::ShowEpsilon, nullptr }, { &CRANSAC::ShowResultAngle, &CRANSAC::ShowResultDistance, nullptr }, planeLabels },
	{ "Perpendicular Plane", pcl::SACMODEL_PERPENDICULAR_PLANE, false, { &CRANSAC::ShowAxis, &CRANSAC::ShowEpsilon, nullptr }, { &CRANSAC::ShowResultAngle, &CRANSAC::ShowResultDistance, nullptr }, planeLabels },
	{ "Stick", pcl::SACMODEL_STICK, false, { &CRANSAC::ShowRadiusLimits, nullptr }, { nullptr }, lineLabels },
	//	{ "Cylinder", pcl::SACMODEL_CYLINDER, false, { &CRANSAC::ShowRadiusLimits, &CRANSAC::ShowAxis, &CRANSAC::m_fEpsilon, nullptr }, cylinderLabels },
	//	{ "Cone", pcl::SACMODEL_CONE, false, { &CRANSAC::m_fAxis, &CRANSAC::m_fEpsilon, &CRANSAC::m_fConeAngle, nullptr }, { &CRANSAC::ShowResultAngle, &CRANSAC::ShowResultDistanceXZ, nullptr }, coneLabels },
	//	{ "Torus", pcl::SACMODEL_TORUS, false, { nullptr } },
	////	{ "Parallel Lines", pcl::SACMODEL_PARALLEL_LINES, false, { nullptr } },
	//	{ "Registration", pcl::SACMODEL_REGISTRATION, { nullptr } },
	//	{ "Registration 2D", pcl::SACMODEL_REGISTRATION_2D, false, { nullptr } },
};

CRANSAC::CRANSAC()
{
	Reset();
}


bool CRANSAC::FindFeatures(const rs2::points& rs2_points)
{
	if (m_bBusy)
		return false;

	if (!rs2_points)
		return false;

	if (!processing_thread)
	{
		processing_thread = new boost::thread([&]() {
			while (1)
			{
				work_to_do.wait();

				if (m_bStop)
					return;
				{
					XPRINT("PointCloud passed to CRANSAC::FindFeatures: " << pointCloud->size() << " data points\n");

					norm_ptr normalCloud(new pcl::PointCloud<pcl::Normal>);
					pcl_ptr newpoints(new pcl::PointCloud<pcl::PointXYZ>);
					norm_ptr newnormals(new pcl::PointCloud<pcl::Normal>);

					if (usesNormals)
					{
						// estimate normals

						pcl::IntegralImageNormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
						ne.setNormalEstimationMethod(ne.AVERAGE_3D_GRADIENT);
						ne.setMaxDepthChangeFactor(0.02f);
						ne.setNormalSmoothingSize(10.0f);
						ne.setInputCloud(pointCloud);
						ne.compute(*normalCloud);


						FilterCloudAndNormals(pointCloud, normalCloud, newpoints, newnormals);
						pointCloud = newpoints;
						normalCloud = newnormals;
					}

					XPRINT("After filtering: " << pointCloud->size() << " data points\n");

					pcl::SacModel sacModel = Models[m_nModel].model;

					pcl_ptr workingPointCloud(new pcl::PointCloud<pcl::PointXYZ>);					// Working set of points
					*workingPointCloud = *pointCloud;

					pcl_ptr cloud_f(new pcl::PointCloud<pcl::PointXYZ>);							// Temp for updating working set

					// Now run RANSAC using the SAC model and parameters

					pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
					pcl::PointIndices::Ptr inliers(new pcl::PointIndices());

					// Create the segmentation object
					pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> segN;
					pcl::SACSegmentation<pcl::PointXYZ>& seg = usesNormals ? segN : *(new pcl::SACSegmentation<pcl::PointXYZ>);

					// Optional
					seg.setOptimizeCoefficients(true);
					// Mandatory


					seg.setModelType(sacModel);

					seg.setMethodType(m_nMethod);
					seg.setMaxIterations(m_nIterations);
					seg.setDistanceThreshold(m_fDistanceThreshhold);

					seg.setRadiusLimits(m_fRadiusLimits[0], m_fRadiusLimits[1]);

					seg.setAxis(Eigen::Vector3f(m_fAxis[0], m_fAxis[1], m_fAxis[2]));

					seg.setEpsAngle(m_fEpsilon);

					// Create the filtering objects
					pcl::ExtractIndices<pcl::PointXYZ> extract;
					pcl::ExtractIndices<pcl::Normal> extractNormals;

					// SAC Model Name for printing in log file

					std::string strModelName = Models[m_nModel].Name;

					int i = 0, nr_points = (int)workingPointCloud->points.size();
					m_FoundFeatures.clear();

					// While 30% of the original cloud is still there
					while (workingPointCloud->points.size() > 0.3 * nr_points)
					{
						// Segment the largest planar component from the remaining cloud
						seg.setInputCloud(workingPointCloud);
						if (Models[m_nModel].usesNormals)
							segN.setInputNormals(normalCloud);
						seg.segment(*inliers, *coefficients);
						if (inliers->indices.size() == 0)
						{
							XPRINT("Could not estimate a " << strModelName << " for the given dataset.\n");
							break;
						}

						pcl::PointCloud<pcl::PointXYZ>::Ptr cloudExtracted(new pcl::PointCloud<pcl::PointXYZ>);

						// Extract the inliers
						extract.setInputCloud(workingPointCloud);
						extract.setIndices(inliers);
						extract.setNegative(false);
						extract.filter(*cloudExtracted);

						XPRINT("PointCloud representing the " << strModelName << " component: " << cloudExtracted->width * cloudExtracted->height << std::endl);
						//ConvertToSeconds(liExtract.QuadPart - liStartExtract.QuadPart), strModelName, cloudExtracted->width * cloudExtracted->height);

						XPRINT("Model Coefficients:\n");
						XPRINT(*coefficients);

						feature_ptr p(new CRANSAC_FEATURE(cloudExtracted));
						p->m_Model = sacModel;
						pcl_model_coefficients_ptr c(new pcl::ModelCoefficients(*coefficients));
						p->m_pCoefficients = c;
						m_FoundFeatures.push_back(p);

						// Depending on the SAC Model, print the offset angle and distance
						float fDistance = 0;

						switch (sacModel)
						{
						case pcl::SACMODEL_PLANE:
						case pcl::SACMODEL_PERPENDICULAR_PLANE:
						case pcl::SACMODEL_NORMAL_PLANE:
						case pcl::SACMODEL_PARALLEL_PLANE:
						case pcl::SACMODEL_NORMAL_PARALLEL_PLANE:
							XPRINT("Angle = " << Angle(*coefficients) << " rad (" << Angle(*coefficients) * DEGREES_PER_RADIAN <<
								" deg)   Distance = " << coefficients->values[3] << " m (" << coefficients->values[3] * INCHES_PER_METER << "in)\n");
							break;

						case pcl::SACMODEL_LINE:
							break;

							// The circle only returns the x&y coordinates of the circle center, plus its radius.  Let's try computing the distance
							// as the average Z value of all the points in the circle cloud.

						case pcl::SACMODEL_CIRCLE2D:
							for (auto& p : *cloudExtracted)
							{
								fDistance += p.z;
							}
							fDistance /= cloudExtracted->size();
							XPRINT("Average distance to circle: " << fDistance << "m (" << fDistance * INCHES_PER_METER << " in)\n");
							break;

						case pcl::SACMODEL_CIRCLE3D:
						case pcl::SACMODEL_NORMAL_SPHERE:
						case pcl::SACMODEL_SPHERE:
							XPRINT("Angle = " << Angle(*coefficients) << " rad (" << Angle(*coefficients) * DEGREES_PER_RADIAN <<
								" deg)   Distance = " << DistanceXZ(*coefficients) << " m (" << DistanceXZ(*coefficients) * INCHES_PER_METER << "in)\n");
							break;

						case pcl::SACMODEL_CYLINDER:
							//case pcl::SACMODEL_CONE:
							//case pcl::SACMODEL_TORUS:
						case pcl::SACMODEL_PARALLEL_LINE:
							//case pcl::SACMODEL_PARALLEL_LINES:
						case pcl::SACMODEL_REGISTRATION:
						case pcl::SACMODEL_REGISTRATION_2D:
						case pcl::SACMODEL_STICK:
							break;
						}

						// Create the filtering object

						extract.setNegative(true);
						extract.filter(*cloud_f);
						workingPointCloud.swap(cloud_f);

						if (usesNormals)
						{
							extractNormals.setInputCloud(normalCloud);
							extractNormals.setIndices(inliers);
							extractNormals.setNegative(true);
							extractNormals.filter(*newnormals);
							normalCloud.swap(newnormals);
						}

						i++;
						if (m_FoundFeatures.size() >= (N_COLORS - 1))
							break;
					}
				}

				// If we found some objects, sort them by size and get rid of the "small" ones....

				if (m_FoundFeatures.size())
				{
					::qsort(&m_FoundFeatures[0], m_FoundFeatures.size(), sizeof(m_FoundFeatures[0]), [](const void* p1, const void* p2) -> int {
						const feature_ptr* f1 = (feature_ptr*)p1;
						const feature_ptr* f2 = (feature_ptr*)p2;
						return (*f2)->cloud->size() - (*f1)->cloud->size();				// Sort in descending order
					});

				int sizeLargeEnough = 0.1 * m_FoundFeatures[0]->cloud->size();
				int nOrigLayers = m_FoundFeatures.size();
				for (int i=0; i< nOrigLayers; i++)
					if (m_FoundFeatures[i]->cloud->size() < sizeLargeEnough)
					{
						m_FoundFeatures.resize(i);
						XPRINT("Removed " << (nOrigLayers - i) << " of " << nOrigLayers << std::endl);
						break;
					}

				}

				m_DisplayUpdateLock.lock();
				m_DisplayFeatures = m_FoundFeatures;
				m_DisplayUpdateLock.unlock();

				m_bBusy = false;

				if (m_bStop)
					return;
			}
		});
	}

	if (m_nModel != -1)
	{
		m_bBusy = true;

		usesNormals = Models[m_nModel].usesNormals;
		pointCloud = points_to_pcl(rs2_points, usesNormals);
		work_to_do.notify();

		return true;
	}

	return false;
}

void CRANSAC::RenderUI()
{
	m_DisplayUpdateLock.lock();

	ImGui::Begin("RANSAC Results");
	{

		auto model = Models[m_nModel];

		for (int nLayer = 0; nLayer < m_DisplayFeatures.size(); nLayer++)
		{
			auto& layer = m_DisplayFeatures[nLayer];
			auto color = LayerColors[nLayer];
			ImGui::TextColored(IM_COLOR(color), "# pts:   %6d points", layer->cloud->size());

			for (int i = 0; i < layer->m_pCoefficients->values.size(); i++)
			{
				ImGui::TextColored(IM_COLOR(color), "%-9s%6.2f m    (%5.1f in)", (std::string(model.coefLabels[i]) + ":").c_str(), layer->m_pCoefficients->values[i], layer->m_pCoefficients->values[i] * INCHES_PER_METER);
			}
			for (void (CRANSAC::* rptr)(int) : model.resultFunctions)
			{
				if (rptr != nullptr)
					(this->*rptr)(nLayer);
				else
					break;
			}
		}
	}
	ImGui::End();
	m_DisplayUpdateLock.unlock();

	ImGui::Begin("RANSAC Parameters");
	{
		ImGui::PushStyleVar(ImGuiStyleVar_FrameRounding, 12);
		ImGui::PushStyleColor(ImGuiCol_SliderGrab, { 40 / 255.f, 170 / 255.f, 90 / 255.f, 1 });
		ImGui::PushStyleColor(ImGuiCol_SliderGrabActive, { 20 / 255.f, 150 / 255.f, 70 / 255.f, 1 });
		ImGui::GetStyle().GrabRounding = 12;

		if (ImGui::Combo("SAC Method", &m_nMethod, &MethodInfo::ItemGetter, Methods, IM_ARRAYSIZE(Methods)))
			g_settings.SetValue("RANSAC", "Method", m_nMethod);

		if (ImGui::Combo("SAC Model", &m_nModel, &ModelInfo::ItemGetter, Models, IM_ARRAYSIZE(Models)))
			g_settings.SetValue("RANSAC", "Model", m_nModel);

		if (ImGui::SliderInt("Number of Iterations", &m_nIterations, 100, 5000))
			g_settings.SetValue("RANSAC", "Iterations", m_nIterations);

		if (ImGui::SliderFloat("Distance Threshhold", &m_fDistanceThreshhold, 0, 0.1))
			g_settings.SetValue("RANSAC", "DistanceThreshhold", m_fDistanceThreshhold);

		if (m_nModel >= 0 && m_nModel < IM_ARRAYSIZE(Models))
		{
			ModelInfo& mi = Models[m_nModel];
			for (void (CRANSAC::* fptr)() : mi.paramFunctions)
			{
				if (fptr != nullptr)
					(this->*fptr)();
				else
					break;
			}
		}
		ImGui::PopStyleColor(2);
		ImGui::PopStyleVar();
	}
	ImGui::End();

}

void CRANSAC::ShowRadiusLimits()
{
	if (ImGui::SliderFloat2("Radius Limits (min/max)", m_fRadiusLimits, 0.0, 1.0))
	{
		g_settings.SetValue("RANSAC", "RadiusLimits0", m_fRadiusLimits[0]);
		g_settings.SetValue("RANSAC", "RadiusLimits1", m_fRadiusLimits[1]);
	}
}

void CRANSAC::ShowAxis()
{
	if (ImGui::SliderFloat3("Axis (x/y/z)", m_fAxis, 0.0, 1.0))
	{
		g_settings.SetValue("RANSAC", "Axis0", m_fAxis[0]);
		g_settings.SetValue("RANSAC", "Axis1", m_fAxis[1]);
		g_settings.SetValue("RANSAC", "Axis2", m_fAxis[2]);
	}
}

void CRANSAC::ShowEpsilon()
{
	if (ImGui::SliderFloat("Epsilon", &m_fEpsilon, 0.0, 1.0))
		g_settings.SetValue("RANSAC", "Epsilon", m_fEpsilon);
}

void CRANSAC::ShowConeAngle()
{
	if (ImGui::SliderFloat2("Cone Angle (min/max)", m_fConeAngle, 0.0, 3.1415926))
	{
		g_settings.SetValue("RANSAC", "ConeAngle0", m_fConeAngle[0]);
		g_settings.SetValue("RANSAC", "ConeAngle1", m_fConeAngle[1]);
	}
}

void CRANSAC::ShowResultDistance(int nLayer)
{
	auto d = m_DisplayFeatures[nLayer]->m_pCoefficients->values[3];
	auto color = LayerColors[nLayer];
	ImGui::TextColored(IM_COLOR(color), "Distance: %5.2f m    (%5.1f in)", d, d * INCHES_PER_METER);
}

void CRANSAC::ShowResultAngle(int nLayer)
{
	auto a = Angle(*m_DisplayFeatures[nLayer]->m_pCoefficients);
	auto color = LayerColors[nLayer];
	ImGui::TextColored(IM_COLOR(color), "Angle:    %5.2f rad  (%5.1f deg)", a, a* DEGREES_PER_RADIAN);
}

void CRANSAC::ShowResultDistanceXZ(int nLayer)
{
	auto d = DistanceXZ(*m_DisplayFeatures[nLayer]->m_pCoefficients);
	auto color = LayerColors[nLayer];
	ImGui::TextColored(IM_COLOR(color), "Distance: %5.2f m    (%5.1f in)", d, d * INCHES_PER_METER);
}

void CRANSAC::ShowResultAvgDistance(int nLayer)
{
	float fDistance = 0.0f;
	for (auto& p : *m_DisplayFeatures[nLayer]->cloud)
	{
		fDistance += p.z;
	}
	fDistance /= m_DisplayFeatures[nLayer]->cloud->size();

	auto color = LayerColors[nLayer];
	ImGui::TextColored(IM_COLOR(color), "Distance: %5.2f m    (%5.1f in)", fDistance, fDistance * INCHES_PER_METER);
}

void CRANSAC::RegisterSettings()
{
	g_settings.SetValue("RANSAC", "Method", m_nMethod);
	g_settings.SetValue("RANSAC", "Model", m_nModel);
	g_settings.SetValue("RANSAC", "Iterations", m_nIterations);
	g_settings.SetValue("RANSAC", "DistanceThreshhold", m_fDistanceThreshhold);
	g_settings.SetValue("RANSAC", "RadiusLimits0", m_fRadiusLimits[0]);
	g_settings.SetValue("RANSAC", "RadiusLimits1", m_fRadiusLimits[1]);
	g_settings.SetValue("RANSAC", "Axis0", m_fAxis[0]);
	g_settings.SetValue("RANSAC", "Axis1", m_fAxis[1]);
	g_settings.SetValue("RANSAC", "Axis2", m_fAxis[2]);
	g_settings.SetValue("RANSAC", "Epsilon", m_fEpsilon);
	g_settings.SetValue("RANSAC", "ConeAngle0", m_fConeAngle[0]);
	g_settings.SetValue("RANSAC", "ConeAngle1", m_fConeAngle[1]);
}

void CRANSAC::ImportSettings()
{
	m_nMethod = g_settings.GetValue<int>("RANSAC", "Method", 0);
	m_nModel = g_settings.GetValue<int>("RANSAC", "Model", 0);
	m_nIterations = g_settings.GetValue<int>("RANSAC", "Iterations", 1000);
	m_fDistanceThreshhold = g_settings.GetValue("RANSAC", "DistanceThreshhold", 0.0f);
	m_fRadiusLimits[0] = g_settings.GetValue("RANSAC", "RadiusLimits0", 0.0f);
	m_fRadiusLimits[1] = g_settings.GetValue("RANSAC", "RadiusLimits1", 0.0f);
	m_fAxis[0] = g_settings.GetValue("RANSAC", "Axis0", 0.0f);
	m_fAxis[1] = g_settings.GetValue("RANSAC", "Axis1", 0.0f);
	m_fAxis[2] = g_settings.GetValue("RANSAC", "Axis2", 0.0f);
	m_fEpsilon = g_settings.GetValue("RANSAC", "Epsilon", 0.0f);
	m_fConeAngle[0] = g_settings.GetValue("RANSAC", "ConeAngle0", 0.0f);
	m_fConeAngle[1] = g_settings.GetValue("RANSAC", "ConeAngle1", 0.0f);
}

void CRANSAC::Reset()
{
	m_nModel = 0;
	m_nIterations = 1000;
	m_fDistanceThreshhold = 0;
	m_fRadiusLimits[0] = 0;
	m_fRadiusLimits[1] = 0;
	m_fAxis[0] = 0;
	m_fAxis[1] = 0;
	m_fAxis[2] = 0;
	m_fEpsilon = 0.0;
	m_fConeAngle[0] = 0;
	m_fConeAngle[1] = 0;
}

std::ostream& operator<<(std::ostream os, const CRANSAC_FEATURE feature)
{
	char Buffer[10000];
	const pcl::ModelCoefficients& coefficients = *feature.m_pCoefficients;
	float fDistance = 0;

	switch (feature.m_Model)
	{
	case pcl::SACMODEL_PLANE:
	case pcl::SACMODEL_PERPENDICULAR_PLANE:
	case pcl::SACMODEL_NORMAL_PLANE:
	case pcl::SACMODEL_PARALLEL_PLANE:
	case pcl::SACMODEL_NORMAL_PARALLEL_PLANE:
		sprintf_s(Buffer, sizeof(Buffer), "Angle = %.2f rad (%.1f deg)   Distance = %.2f m (%.1f in)n",
			Angle(coefficients), Angle(coefficients) * DEGREES_PER_RADIAN, coefficients.values[3], coefficients.values[3] * INCHES_PER_METER);
		break;

	case pcl::SACMODEL_LINE:
	case pcl::SACMODEL_CIRCLE3D:
		break;

		// The circle only returns the x&y coordinates of the circle center, plus its radius.  Let's try computing the distance
		// as the average Z value of all the points in the circle cloud.

	case pcl::SACMODEL_CIRCLE2D:
		//for (auto& p : *cloudExtracted)
		//{
		//	fDistance += p.z;
		//}
		//fDistance /= cloudExtracted->size();
		sprintf_s(Buffer, sizeof(Buffer), "NEED TO UNCOMMENT CODE: Average distance to circle: %.2f (%.1f in)\n", fDistance, fDistance * INCHES_PER_METER);
		break;

	case pcl::SACMODEL_SPHERE:
		sprintf_s(Buffer, sizeof(Buffer), "Angle = %.2f rad (%.1f deg)   Distance = %.2f m (%.1f in)\n",
			Angle(coefficients), Angle(coefficients) * DEGREES_PER_RADIAN, DistanceXZ(coefficients), DistanceXZ(coefficients) * INCHES_PER_METER);
		break;

	case pcl::SACMODEL_CYLINDER:
		//case pcl::SACMODEL_CONE:
		//case pcl::SACMODEL_TORUS:
	case pcl::SACMODEL_PARALLEL_LINE:
		//case pcl::SACMODEL_PARALLEL_LINES:
	case pcl::SACMODEL_NORMAL_SPHERE:
	case pcl::SACMODEL_REGISTRATION:
	case pcl::SACMODEL_REGISTRATION_2D:
	case pcl::SACMODEL_STICK:
		break;
	}

	return os;
}
