#include "all_includes.h"

#include "PCLUtils.h"

double
DistanceXZ(const pcl::ModelCoefficients& v)
{
	return sqrt(v.values[0] * v.values[0] + v.values[2] * v.values[2]);
}

double
Angle(const pcl::ModelCoefficients& v)
{
	return asin(v.values[0] / DistanceXZ(v));
}



pcl_ptr points_to_pcl(const rs2::points& points, bool bIntegralImage)
{
//	std::tuple<uint8_t, uint8_t, uint8_t> RGB_Color;
	pcl_ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

	auto sp = points.get_profile().as<rs2::video_stream_profile>();
	cloud->width = sp.width();
	cloud->height = sp.height();
	cloud->is_dense = false;
	if (bIntegralImage)
		cloud->points.resize(points.size());

	auto Texture_Coord = points.get_texture_coordinates();
	auto Vertex = points.get_vertices();
	auto ptr = Vertex;

		//===================================
		// Mapping Depth Coordinates
		// - Depth data stored as XYZ values
		//===================================

	if (bIntegralImage)
	{
		for (int i = 0; i < points.size(); i++)
		{
			cloud->points[i].x = Vertex[i].x;
			cloud->points[i].y = Vertex[i].y;
			cloud->points[i].z = Vertex[i].z;
		}
	}
	else
	{
		for (int i = 0; i < points.size(); i++)
		{
			if (Vertex[i].z > 0.0)
				cloud->push_back(pcl::PointXYZ(Vertex[i].x, Vertex[i].y, Vertex[i].z));
		}
	}

	return cloud;
}

void FilterCloudAndNormals(const pcl_ptr points, const norm_ptr normals, pcl_ptr& newpoints, norm_ptr& newnormals)
{
	//newpoints = new pcl::PointCloud <pcl::PointXYZ>;
	//newnormals = new pcl::PointCloud<pcl::Normal>;

	newpoints->width = newnormals->width = points->width;
	newpoints->height = newnormals->height = points->height;
	newpoints->is_dense = newnormals->is_dense = false;

	auto Vertex = points->data();
	auto Normal = normals->data();

	auto n = Normal[0];

	//===================================
	// Mapping Depth Coordinates
	// - Depth data stored as XYZ values
	//===================================

	for (int i = 0; i < points->width * points->height; i++)
	{
		if (Vertex[i].z > 0.0) {
			newpoints->push_back(pcl::PointXYZ(Vertex[i].x, Vertex[i].y, Vertex[i].z));
			newnormals->push_back(pcl::Normal(Normal[i].normal_x, Normal[i].normal_y, Normal[i].normal_z, Normal[i].curvature));
		}
	}
}

//std::ostream& operator<<(std::ostream os, const pcl::ModelCoefficients& v)
//{
//	os << "seq: " << v.header.seq << " stamp: " << v.header.stamp;
//	os << "  frame_id: " << v.header.frame_id << std::endl;
//	os << "values[]\n";
//	for (size_t i = 0; i < v.values.size(); ++i)
//	{
//		os << "  values[" << i << "]   " << v.values[i] << " m    " << v.values[i] * INCHES_PER_METER << " in\n");
//	}
//
//	return os;
//}

