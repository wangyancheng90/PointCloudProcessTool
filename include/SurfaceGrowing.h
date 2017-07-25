/****************************************************************************************************************************************
File name: SurfaceGrowing.h
Author: Yancheng Wang
Version: 1.0
Date: 2017-7-15
Description: SurfaceGrow segmentation for point cloud to dectect plane or smooth curved surface
****************************************************************************************************************************************/
#pragma once
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/PointIndices.h>
#include "WYCPlane.h"
#include "WYCointType.hpp"

struct _SurfaceGrowingParameters
{
	int seed_min_number;
	double seed_max_distance;
	double seed_search_radius;
	int segment_min_number;
	double segment_max_distance;
	double segment_search_nearest_k;
	double segment_search_radius;

	double recompute_min_distance;
	int recompute_min_number;
	bool do_plane_detect;
	bool do_compete;
	bool do_verbose;
};

struct SurfaceGrowingParameters2 :public _SurfaceGrowingParameters
{
	inline SurfaceGrowingParameters2()
	{
		seed_min_number = 15;
		seed_max_distance = 0.05;
		seed_search_radius = 0.5;
		segment_min_number = 50;
		segment_max_distance = 0.15;
		segment_search_nearest_k = 100;
		segment_search_radius = 0.3;
		recompute_min_distance = 0.15;
		recompute_min_number = 10; 
		do_plane_detect = true;
		do_compete = true;
		do_verbose = false;
	}
};

class SurfaceGrow
{
public:
	int GetAllSegments(boost::shared_ptr<pcl::PointCloud<PointT> > cloud, vector<pcl::PointIndices>& pointindex_per_tagvalue, PointAttributeTag tag);
	void SetAttribute(boost::shared_ptr<pcl::PointCloud<PointT> > cloud, PointAttributeTag tag, int value);
	int SurfaceGrowing(pcl::PointCloud<PointT>::Ptr cloud, const SurfaceGrowingParameters2& paras, vector<pcl::PointIndices>& segment_indices, vector<WYCPlane>& planes_out, int start_value=1);
};
