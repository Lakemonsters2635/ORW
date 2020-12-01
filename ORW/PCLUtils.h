#pragma once

#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API

#include <pcl/sample_consensus/sac_model.h>
#include <pcl/ModelCoefficients.h>

#include "Utilities.h"

using pcl_ptr = pcl::PointCloud<pcl::PointXYZ>::Ptr;
using pcl_color_ptr = pcl::PointCloud<pcl::PointXYZRGBA>::Ptr;
using pcl_model_coefficients_ptr = pcl::ModelCoefficients::Ptr;
using norm_ptr = pcl::PointCloud<pcl::Normal>::Ptr;


double DistanceXZ(const pcl::ModelCoefficients& v);
double Angle(const pcl::ModelCoefficients& v);

pcl_ptr points_to_pcl(const rs2::points& points, bool bIntegralImage = false);

void FilterCloudAndNormals(const pcl_ptr points, const norm_ptr normals, pcl_ptr& newpoints, norm_ptr& newnormals);
