#include<iostream>
#include <pcl/search/kdtree.h>
#include "SurfaceGrowing.h"
#include "WYCPointType.hpp"
#include "HoughTransform3D.h"
//#include "wycio.hpp"
using namespace std;

int SurfaceGrow::GetAllSegments(boost::shared_ptr<pcl::PointCloud<PointT> > cloud, vector<pcl::PointIndices>& pointindex_per_tagvalue, PointAttributeTag tag)
{
	pointindex_per_tagvalue.clear();
	vector<int> tagvalues;
	vector<int>::iterator tagvalues_itr;
	tagvalues.clear();
	bool found;
	int value;
	int index_in_tagvalues;
	if (tag == 1)
	{
		for (int i = 0; i < cloud->points.size(); i++)
		{
			found = false;
			value = cloud->points.at(i).LabelTag;
			if (value == 0) continue;
			index_in_tagvalues = -1;
			//check the value is already exist or not
			for (tagvalues_itr = tagvalues.begin(), found = false; tagvalues_itr != tagvalues.end() && !found; tagvalues_itr++)
			{
				if (*tagvalues_itr == value)
				{
					found = true;
				}
				index_in_tagvalues++;
			}
			if (found)
			{
				pointindex_per_tagvalue.at(index_in_tagvalues).indices.push_back(i);
			}
			else
			{
				pcl::PointIndices new_point_indices;
				new_point_indices.header.seq = value;
				new_point_indices.header.frame_id = "LabelTag";
				new_point_indices.indices.resize(1);
				new_point_indices.indices.at(0) = i;
				pointindex_per_tagvalue.push_back(new_point_indices);
				tagvalues.push_back(value);

			}
		}
	}

	else if (tag == 2)
	{
		for (int i = 0; i < cloud->points.size(); i++)
		{
			found = false;
			value = cloud->points.at(i).SegmentNumberTag;
			if (value == 0) continue;
			index_in_tagvalues = -1;
			//check the value is already exist or not
			for (tagvalues_itr = tagvalues.begin(), found = false; tagvalues_itr != tagvalues.end() && !found; tagvalues_itr++)
			{
				if (*tagvalues_itr == value)
				{
					found = true;
				}
				index_in_tagvalues++;
			}
			if (found)
			{
				pointindex_per_tagvalue.at(index_in_tagvalues).indices.push_back(i);
			}
			else
			{
				pcl::PointIndices new_point_indices;
				new_point_indices.header.seq = value;
				new_point_indices.header.frame_id = "SegmentNumberTag";
				new_point_indices.indices.resize(1);
				new_point_indices.indices.at(0) = i;
				pointindex_per_tagvalue.push_back(new_point_indices);
				tagvalues.push_back(value);

			}
		}
	}
	else if (tag == 3)
	{
		for (int i = 0; i < cloud->points.size(); i++)
		{
			found = false;
			value = cloud->points.at(i).UserIntTag1;
			if (value == 0) continue;
			index_in_tagvalues = -1;
			//check the value is already exist or not
			for (tagvalues_itr = tagvalues.begin(), found = false; tagvalues_itr != tagvalues.end() && !found; tagvalues_itr++)
			{
				if (*tagvalues_itr == value)
				{
					found = true;
				}
				index_in_tagvalues++;
			}
			if (found)
			{
				pointindex_per_tagvalue.at(index_in_tagvalues).indices.push_back(i);
			}
			else
			{
				pcl::PointIndices new_point_indices;
				new_point_indices.header.seq = value;
				new_point_indices.header.frame_id = "UserIntTag1";
				new_point_indices.indices.resize(1);
				new_point_indices.indices.at(0) = i;
				pointindex_per_tagvalue.push_back(new_point_indices);
				tagvalues.push_back(value);

			}
		}
	}
	else if (tag == 6)
	{
		for (int i = 0; i < cloud->points.size(); i++)
		{
			found = false;
			value = cloud->points.at(i).UserData;
			if (value == 0) continue;
			index_in_tagvalues = -1;
			//check the value is already exist or not
			for (tagvalues_itr = tagvalues.begin(), found = false; tagvalues_itr != tagvalues.end() && !found; tagvalues_itr++)
			{
				if (*tagvalues_itr == value)
				{
					found = true;
				}
				index_in_tagvalues++;
			}
			if (found)
			{
				pointindex_per_tagvalue.at(index_in_tagvalues).indices.push_back(i);
			}
			else
			{
				pcl::PointIndices new_point_indices;
				new_point_indices.header.seq = value;
				new_point_indices.header.frame_id = "UserData";
				new_point_indices.indices.resize(1);
				new_point_indices.indices.at(0) = i;
				pointindex_per_tagvalue.push_back(new_point_indices);
				tagvalues.push_back(value);

			}
		}
	}
	else
	{
		cout << "can not identify the int tag, please check the spelling" << endl;
		return -1;
	}
	return pointindex_per_tagvalue.size();
}

void SurfaceGrow::SetAttribute(boost::shared_ptr<pcl::PointCloud<PointT> > cloud, PointAttributeTag tag, int value)
{
	if (tag == 1)
		for (int i = 0; i < cloud->size(); i++)
			cloud->points.at(i).LabelTag = value;

	else if (tag == 2)
		for (int i = 0; i < cloud->size(); i++)
			cloud->points.at(i).SegmentNumberTag = value;

	else if (tag == 3)
		for (int i = 0; i < cloud->size(); i++)
			cloud->points.at(i).UserIntTag1 = value;
	else if (tag == 6)
		for (int i = 0; i < cloud->size(); i++)
			cloud->points.at(i).UserData = value;
	else if (tag == 7)
		for (int i = 0; i < cloud->size(); i++)
			cloud->points.at(i).Classification = value;
	else
	{
		cout << "invalid attribute in SetAttribute, check your spelling!" << endl;
	}
	return;
}

int SurfaceGrow::SurfaceGrowing(pcl::PointCloud<PointT>::Ptr cloud_orig, const SurfaceGrowingParameters2& paras, vector<pcl::PointIndices>& segment_indices, vector<WYCPlane>& planes_out, int start_value)
{
	segment_indices.clear();
	pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
	cloud->points = cloud_orig->points;
	//建KDtree
	pcl::KdTreeFLANN<PointT> kdtree;
	kdtree.setInputCloud(cloud);
	SetAttribute(cloud, SEGMENT_NUMBER_TAG, 0);
	SetAttribute(cloud, USER_INT_TAG1, 0);
	//给每一个点建一个plane
	vector<WYCPlane> planes;
	WYCPlane one_plane;
	if (!paras.do_plane_detect)
	{
		for (int i = 0; i < cloud->size(); i++)
		{
			planes.push_back(one_plane);
		}
	}


	std::vector<int> pointIdx;
	std::vector<float> pointquaredDistance;
	std::vector<int> compete_pointIdx;
	std::vector<float>  compete_pointquaredDistance;
	std::vector<int> recompute_pointIdx;
	std::vector<float> recompute_pointquaredDistance;
	//hough空间的分辨率单位（角度，角度，米）
	Coordinate hough_resolution(1, 1, 0.05);
	Coordinate hough_cancel_window_size(5, 5, 3);
	Coordinate current_base_pt;
	Coordinate current_pt;
	vector<Coordinate> seed_coordinates;
	vector<Coordinate> seed_coordinates_refine1;
	vector<Coordinate> seed_coordinates_refine2;
	vector<Coordinate> coordinates_for_new_plane;
	Coordinate seed_coordinate;
	WYCPlane seed_plane;
	WYCPlane current_plane;
	WYCPlane new_plane;
	bool seed_plane_found = false;
	bool good_seed_plane_found = false;
	bool add_point = false;
	int ignored_planes_number = 0;
	int plane_points_count = 0;
	int good_seed = 0;
	int segment_number = 0;
	int refit_plane_interval = 10;
	HoughTransform3D seed_hough;
	vector<int> seed_indices;

	double current_dist_to_plane = 0;
	double sq_error_old = 0;
	double sq_error_new = 0;
	//开始生长了
	for (int i = 0; i < cloud->size(); i++)
	{
		if (cloud->points[i].SegmentNumberTag == 0 && cloud->points[i].UserIntTag1 == 0)//没有处理过且没有segmentTag的点
		{
			//搜索邻域点，看能否成为种子
			kdtree.radiusSearch(cloud->points[i], paras.seed_search_radius, pointIdx, pointquaredDistance, 200);
			good_seed = 0;
			seed_coordinates.clear();
			for (int j = 0; j < pointIdx.size(); j++)
			{
				if (cloud->points[pointIdx[j]].SegmentNumberTag == 0)
				{
					pointIdx[good_seed] = pointIdx[j];
					seed_coordinate.x = cloud->points[pointIdx[j]].x;
					seed_coordinate.y = cloud->points[pointIdx[j]].y;
					seed_coordinate.z = cloud->points[pointIdx[j]].z;
					seed_coordinates.push_back(seed_coordinate);
					good_seed++;
				}
			}
			pointIdx.resize(good_seed);
			//点数判断能不能成为种子
			if (good_seed<paras.seed_min_number) continue;

			//开始平面判断
		//	cout << "seed_coordinates.size(): " << seed_coordinates.size() << endl;
			 seed_hough.Initialise(seed_coordinates, hough_resolution);
			 seed_plane_found = false;
			 good_seed_plane_found = false;
			 ignored_planes_number = 0;
			 plane_points_count = 0;
			do
			{
				// 第一次 hough变换计算出一个平面best_seed_plane
				seed_plane = seed_hough.GetBestPlane(ignored_planes_number, hough_cancel_window_size, plane_points_count);
			//	cout << "Hough plane_points_count: " << plane_points_count << " ignored_planes_number: " << ignored_planes_number<< endl;
				seed_plane_found = (plane_points_count >= paras.seed_min_number);
				if (seed_plane_found)
				{
					//seed_coordinates里离best_seed_plane够近的点个数,够近的点再cloud里，将userdoubletag1标记维1，否则，标记为0
					plane_points_count = 0;
					seed_coordinates_refine1.clear();
					for (int j = 0; j < pointIdx.size(); j++)
					{
						if (fabs(seed_plane.Distance(seed_coordinates[j])) <= paras.seed_max_distance)
						{
							seed_coordinates_refine1.push_back(seed_coordinates[j]);
							plane_points_count++;
						}
						else
						{
							NULL;
						}
					}
					seed_plane_found = (plane_points_count >= paras.seed_min_number);
					if (seed_plane_found)
					{
		//				cout << "2 ";
						// Improve the plane fit， 第二次 用第一次的平面找周围的点，再用最小二乘拟合一个平面
						seed_plane.Initialise(seed_coordinates_refine1);

						//seed_coordinates里离plane_refine1够近的点个数,够近的点再cloud里，将userdoubletag1标记维1，否则，标记为0
						seed_coordinates_refine2.clear();
						plane_points_count = 0;
						for (int j = 0; j < pointIdx.size(); j++)
						{
							if (fabs(seed_plane.Distance(seed_coordinates[j])) <= paras.seed_max_distance)
							{
								seed_coordinates_refine2.push_back(seed_coordinates[j]);
								plane_points_count++;
							}
							else
							{
								NULL;
							}
						}

						//完后再fit一遍plane,再找离平面近的点
						seed_plane.Initialise(seed_coordinates_refine2);
						//seed_coordinates里离plane_refine2够近的点个数,够近的点再cloud里，将userdoubletag1标记维1，否则，标记为0
						plane_points_count = 0;
						seed_indices.clear();
						for (int j = 0; j < pointIdx.size(); j++)
						{
							if (fabs(seed_plane.Distance(seed_coordinates[j])) <= paras.seed_max_distance)
							{
								seed_indices.push_back(pointIdx[j]);
								seed_coordinates_refine2.push_back(seed_coordinates[j]);
								plane_points_count++;
							}
							else
							{
								NULL;
							}
						}
						seed_plane_found = (plane_points_count>= paras.seed_min_number);
						current_base_pt.x = cloud->points[i].x;
						current_base_pt.y = cloud->points[i].y;
						current_base_pt.z = cloud->points[i].z;
						good_seed_plane_found = (fabs(seed_plane.Distance(current_base_pt)) <= paras.seed_max_distance);
						if (!good_seed_plane_found) ignored_planes_number++;
					}
				}
			} while (!good_seed_plane_found && seed_plane_found&&ignored_planes_number<5);
	//		cout << "2 ";
			if (good_seed_plane_found)//如果找到了好的seed平面
			{
				segment_number++;
				seed_plane.SetSegmentNumber(start_value - 1 + segment_number);
				//记录plane信息
				if (paras.do_plane_detect)//检测平面
				{
					planes.push_back(seed_plane);
			//		seed_plane.Print();
				//	system("PAUSE");
					current_plane = planes[planes.size()-1];
	//				cout << "planes size: " << planes.size() << ", ";
				}
				else//检测连续光滑曲面
				{
					for (int j = 0; j < seed_indices.size(); j++)
					{
						planes[seed_indices[j]] = seed_plane;
					}

				}
		//		cout << "3 ";
				//打tag
				for (int j = 0; j < seed_indices.size(); j++)
				{
					cloud->points[seed_indices[j]].SegmentNumberTag = start_value-1+segment_number;
				}
				refit_plane_interval = 10;
				//开始生长种子
		//		cout << "4 ";
		//		cout << "segment_number: " << segment_number << ", " << start_value - 1 + segment_number<<", ";
				for (int j = 0; j < seed_indices.size(); j++)
				{
					//对每个种子点进行搜索邻域点
					kdtree.radiusSearch(cloud->points[seed_indices[j]], paras.segment_search_radius, pointIdx, pointquaredDistance, paras.segment_search_nearest_k);
					for (int n = 0; n < pointIdx.size(); n++)//对于搜索到的每一个点
					{
				//		cout << "4.5 ";
						if (!paras.do_plane_detect)//如果是检测光滑曲面，则current_plane为当前进行领域搜索的种子点的plane
						{
							current_plane = planes[seed_indices[j]];
						}
						//遍历搜索到的领域点, 如果当前领域点没有segmentnumbertag或者需要做compete, 则进行判断该点是否被生长
			//			cout << "4.8 ";
						if (cloud->points[pointIdx[n]].SegmentNumberTag == 0 ||
							(paras.do_compete&&paras.do_plane_detect&&cloud->points[pointIdx[n]].SegmentNumberTag != start_value - 1 + segment_number))
						{
							current_pt.x = cloud->points[pointIdx[n]].x;
							current_pt.y = cloud->points[pointIdx[n]].y;
							current_pt.z = cloud->points[pointIdx[n]].z;
							current_dist_to_plane = fabs(current_plane.Distance(current_pt));
							//如果距距离当前平面的距离小于阈值
							add_point = false;
					//		cout << "5 ";
							if (current_dist_to_plane <= paras.segment_max_distance)
							{
								add_point = true;
								//开始竞争判断
								if (cloud->points[pointIdx[n]].SegmentNumberTag != 0)
								{
									kdtree.radiusSearch(cloud->points[pointIdx[n]], paras.segment_search_radius, compete_pointIdx, compete_pointquaredDistance, paras.segment_search_nearest_k*9);
								/*	int compete_count_old = 0;
									int compete_count_new = 0;*/
									sq_error_old = 0;
									sq_error_new = 0;
									for (int m = 0; m < compete_pointIdx.size(); m++)
									{
										//比较新旧两个plane对于它临近点的贴合情况，取RMS小的那个  
										//注意看这里，对于 compete_pointIdx也不是全用，还得从里面选取属于新旧两个plane的点来算dist 
										if (cloud->points[compete_pointIdx[m]].SegmentNumberTag == cloud->points[pointIdx[n]].SegmentNumberTag || cloud->points[compete_pointIdx[m]].SegmentNumberTag == start_value - 1 + segment_number)
										{
											/*	compete_count_old += cloud->points[compete_pointIdx[m]].SegmentNumberTag == cloud->points[pointIdx[n]].SegmentNumberTag;
												compete_count_new += cloud->points[compete_pointIdx[m]].SegmentNumberTag == start_value - 1 + segment_number;*/
											current_pt.x = cloud->points[compete_pointIdx[m]].x;
											current_pt.y = cloud->points[compete_pointIdx[m]].y;
											current_pt.z = cloud->points[compete_pointIdx[m]].z;
											//被竞争点的邻域点到老平面的距离
											current_dist_to_plane = planes[cloud->points[pointIdx[n]].SegmentNumberTag - start_value].Distance(current_pt);
											sq_error_old += current_dist_to_plane*current_dist_to_plane;
											current_dist_to_plane = current_plane.Distance(current_pt);
											sq_error_new += current_dist_to_plane*current_dist_to_plane;
										}
									}
					/*				if (compete_count_old == 0) sq_error_old = DBL_MAX;
									if (sq_error_new == 0) sq_error_new = DBL_MAX-1;*/
									if (sq_error_old < sq_error_new) add_point = false;
										//如果需要更换竞争点的segmentnumbertag，则老平面需要更新
									if (add_point)
									{
										current_pt.x = cloud->points[pointIdx[n]].x;
										current_pt.y = cloud->points[pointIdx[n]].y;
										current_pt.z = cloud->points[pointIdx[n]].z;
									//	planes[cloud->points[pointIdx[n]].SegmentNumberTag - start_value].Print();
									//	system("PAUSE");
										planes[cloud->points[pointIdx[n]].SegmentNumberTag - start_value].RemovePoint(current_pt, true);
									}
								}//结束竞争
							}//结束是否被生长判断
					//		cout << "6 ";
							if (add_point)
							{
								//如果该点加入，则需要判断是否更新当前plane
								cloud->points[pointIdx[n]].SegmentNumberTag = start_value - 1 + segment_number;
								seed_indices.push_back(pointIdx[n]);
							//	cout << "7 ";
								if (paras.do_plane_detect)//对于平面检测
								{
								//	cout << "8 ";
									current_pt.x = cloud->points[pointIdx[n]].x;
									current_pt.y = cloud->points[pointIdx[n]].y;
									current_pt.z = cloud->points[pointIdx[n]].z;
									current_plane.AddPoint(current_pt, seed_indices.size() == (seed_indices.size() / refit_plane_interval) * refit_plane_interval);
							//		current_plane.AddPoint(current_pt, true);
						//			cout << "8.1 ";
									planes[segment_number-1] = current_plane;
							//		cout << "8.2 ";
									if (seed_indices.size()> 2 * refit_plane_interval) refit_plane_interval *= 2;
					//				cout << "8.3 ";
								}
								else//对于连续曲面
								{
					//				cout << "9 ";
									current_pt.x = cloud->points[pointIdx[n]].x;
									current_pt.y = cloud->points[pointIdx[n]].y;
									current_pt.z = cloud->points[pointIdx[n]].z;
									//首先判断这个点到current_plane的距离，如果距离过大，则继续判断
									current_dist_to_plane = current_plane.Distance(current_pt);
									if (current_dist_to_plane > paras.recompute_min_distance)
									{
										//找这个点周围一圈的领域点，如果周围的领域点够多，则重新计算
										kdtree.radiusSearch(cloud->points[pointIdx[n]], paras.segment_search_radius, recompute_pointIdx, recompute_pointquaredDistance, paras.segment_search_nearest_k);
									}
									else recompute_pointIdx.clear();

									
									if (recompute_pointIdx.size() >= paras.recompute_min_number)//如果领域点个数够多
									{
										coordinates_for_new_plane.resize(recompute_pointIdx.size());
										for (int k = 0; k < recompute_pointIdx.size(); k++)
										{
											coordinates_for_new_plane[k].x = cloud->points[recompute_pointIdx[k]].x;
											coordinates_for_new_plane[k].y = cloud->points[recompute_pointIdx[k]].y;
											coordinates_for_new_plane[k].z = cloud->points[recompute_pointIdx[k]].z;
										}
										new_plane.Initialise(coordinates_for_new_plane);
										//新老两个plane的夹角判断
										if (current_plane.GetPlaneNormal().DotProduct(new_plane.GetPlaneNormal())>cos(5 * atan(1.0) / 45))//夹角小于5度，直接用新plane替换
										{
											planes[pointIdx[n]] = new_plane;
										}
										else//否则再原有plane上进行更新
										{
											current_plane.AddPoint(current_pt, true);
											planes[pointIdx[n]] = current_plane;
										}
									}
									else//领域点个数不够多
									{
						//				cout << "10 ";
										planes[pointIdx[n]] = current_plane;
									}
								}//结束 对于曲面的点增长时对面的更新操作
							}//结束 点增长时的操作
						}//结束 一次if判断
					}//结束一次领域搜索的临近点生长判断
				}//结束一个segment的生长
		//		cout << "11 ";
				//生长完毕之后，舍弃掉不够大的segment, 点和plane归零
				if (seed_indices.size() < paras.segment_min_number)
				{
					for (int n = 0; n < seed_indices.size(); n++)
					{
						cloud->points[seed_indices[n]].SegmentNumberTag = 0;
					}
					segment_number--;
					if (paras.do_plane_detect)
					{
						planes[planes.size() - 1].Initialise();
						planes.erase(planes.end() - 1);
					}
				}
	//			cout << "12 ";
			}//结束 好种子的生长
			else//如果没有好的种子，则将这些点注销掉，让他们不参与后续的种子点寻找， 可以加快速度，减少hough变换的次数，但是可能丢失细小的面
			{
	//			cout << "13 ";
				for (int n = 0; n < pointIdx.size(); n++)
				{
					cloud->points[pointIdx[n]].UserIntTag1 = 1;
				}
			}
		}//结束 单个种子点的选取
	}//结束 生长

	//结果保存
	planes_out.swap(planes);
	GetAllSegments(cloud, segment_indices, SEGMENT_NUMBER_TAG);
	for (int i = 0; i < cloud->size(); i++)
	{
		cloud_orig->points[i].SegmentNumberTag = cloud->points[i].SegmentNumberTag;
	}
	return segment_indices.size();
}

