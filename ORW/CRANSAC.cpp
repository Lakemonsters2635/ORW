#include "CRANSAC.h"

#include <gl/glew.h>
#include <GLFW/glfw3.h>
#include "vendor/imgui/imgui_impl_glfw.h"

#include <pcl/sample_consensus/model_types.h>

#include "Utilities.h"

struct ModelInfo
{
	const char* Name;
	pcl::SacModel	model;
	void (CRANSAC::* fptr[3])();

	static bool ItemGetter(void* data, int n, const char** out_str)
	{
		*out_str = ((ModelInfo*)data)[n].Name;
		return true;
	}
};

static ModelInfo Models[] =
{
	{ "None", (pcl::SacModel) -1, { nullptr } },
	{ "Plane", pcl::SACMODEL_PLANE, { nullptr } },
	{ "Line", pcl::SACMODEL_LINE, { nullptr } },
	{ "Circle2D", pcl::SACMODEL_CIRCLE2D, { &CRANSAC::ShowRadiusLimits, nullptr } },
	{ "Circle3D", pcl::SACMODEL_CIRCLE3D, { &CRANSAC::ShowRadiusLimits, nullptr } },
	{ "Sphere", pcl::SACMODEL_SPHERE, { &CRANSAC::ShowRadiusLimits, nullptr } },
	{ "Parallel Line", pcl::SACMODEL_PARALLEL_LINE, { &CRANSAC::ShowAxis, &CRANSAC::ShowEpsilon, nullptr } },
	{ "Parallel Plane", pcl::SACMODEL_PARALLEL_PLANE, { &CRANSAC::ShowAxis, &CRANSAC::ShowEpsilon, nullptr } },
	{ "Perpendicular Plane", pcl::SACMODEL_PERPENDICULAR_PLANE, { &CRANSAC::ShowAxis, &CRANSAC::ShowEpsilon, nullptr } },
	{ "Stick", pcl::SACMODEL_STICK, { &CRANSAC::ShowRadiusLimits, nullptr } },
	//	{ "Cylinder", pcl::SACMODEL_CYLINDER, { &CRANSAC::ShowRadiusLimits, &CRANSAC::ShowAxis, &CRANSAC::m_fEpsilon, nullptr } },
	//	{ "Cone", pcl::SACMODEL_CONE, { &CRANSAC::m_fAxis, &CRANSAC::m_fEpsilon, &CRANSAC::m_fConeAngle, nullptr } },
	//	{ "Torus", pcl::SACMODEL_TORUS, { nullptr } },
	//	{ "Parallel Lines", pcl::SACMODEL_PARALLEL_LINES, { nullptr } },
	//	{ "Normal Plane", pcl::SACMODEL_NORMAL_PLANE, { nullptr } },
	//	{ "Normal Sphere", pcl::SACMODEL_NORMAL_SPHERE, { nullptr } },
	//	{ "Registration", pcl::SACMODEL_REGISTRATION, { nullptr } },
	//	{ "Registration 2D", pcl::SACMODEL_REGISTRATION_2D, { nullptr } },
	//	{ "Normal Parallel Plane", pcl::SACMODEL_NORMAL_PARALLEL_PLANE, { nullptr } },
};

CRANSAC::CRANSAC()
{
	Reset();
}

void CRANSAC::RenderUI()
{
	ImGui::Begin("RANSAC Parameters");
	{
		if (ImGui::Combo("SAC Model", &m_nModel, &ModelInfo::ItemGetter, Models, IM_ARRAYSIZE(Models)))
			g_settings.SetValue("RANSAC", "Model", m_nModel);

		if (ImGui::InputScalar("Number of Iterations", ImGuiDataType_S32, &m_nIterations))
			g_settings.SetValue("RANSAC", "Iterations", m_nIterations);

		if (ImGui::InputScalar("Distance Threshhold", ImGuiDataType_Float, &m_fDistanceThreshhold))
			g_settings.SetValue("RANSAC", "DistanceThreshhold", m_fDistanceThreshhold);

		if (m_nModel >= 0 && m_nModel < IM_ARRAYSIZE(Models))
		{
			ModelInfo& mi = Models[m_nModel];
			for (void (CRANSAC::* fptr)() : mi.fptr)
			{
				if (fptr != nullptr)
					(this->*fptr)();
			}
		}
	}
	ImGui::End();
}

void CRANSAC::ShowRadiusLimits()
{
	if (ImGui::InputFloat2("Radius Limits (min/max)", m_fRadiusLimits))
	{
		g_settings.SetValue("RANSAC", "RadiusLimits0", m_fRadiusLimits[0]);
		g_settings.SetValue("RANSAC", "RadiusLimits1", m_fRadiusLimits[1]);
	}
}

void CRANSAC::ShowAxis()
{
	if (ImGui::InputFloat3("Axis (x/y/z)", m_fAxis))
	{
		g_settings.SetValue("RANSAC", "Axis0", m_fAxis[0]);
		g_settings.SetValue("RANSAC", "Axis1", m_fAxis[1]);
		g_settings.SetValue("RANSAC", "Axis2", m_fAxis[2]);
	}
}

void CRANSAC::ShowEpsilon()
{
	if (ImGui::InputScalar("Epsilon", ImGuiDataType_Float, &m_fEpsilon))
		g_settings.SetValue("RANSAC", "Epsilon", m_fEpsilon);
}

void CRANSAC::ShowConeAngle()
{
	if (ImGui::InputFloat2("Cone Angle (min/max)", m_fConeAngle))
	{
		g_settings.SetValue("RANSAC", "ConeAngle0", m_fConeAngle[0]);
		g_settings.SetValue("RANSAC", "ConeAngle1", m_fConeAngle[1]);
	}
}

void CRANSAC::RegisterSettings()
{
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

