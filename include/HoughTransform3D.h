/****************************************************************************************************************************************
File name: HoughTransform3D.h
Author: Yancheng Wang
Version: 1.0
Date: 2017-7-15
Description: 3D HoughTransform for Point cloud to detect plane
****************************************************************************************************************************************/
#pragma once
#include "Coordinate.h"
#include "WYCPlane.h"

enum HoughEquation
{
	PLANE_EQUATION_1, //sin a sin b X + sin a cos b Y + cos a Z = d
	PLANE_EQUATION_2//Z = slope_X * X + slope_Y * Y + d      
};


class HoughTransform3D
{
protected:
	Coordinate dim_min;//hough 空间的三个维度的坐标最小值
	Coordinate dim_max;//hough 空间的三个维度的坐标最大值
	Coordinate dim_count;// hough 空间的三个维度的格子数， dim_count的坐标值必须都是整数
	Coordinate dim_resolution;//hough 空间的三个维度格子的大小；(内部计算出来的)
	Coordinate offset;//坐标偏移量
	//sin a sin b X + sin a cos b Y + cos a Z = d------(a,b,d）
	//Z = slope_X * X + slope_Y * Y + d    -------(slope_x, slope_y, d)
	HoughEquation equation;//方程

	std::vector<std::vector<std::vector<int> > > hough_space;
	std::vector<double> sin_a;//用于存储对应维度的hough空间小格子中心点的sin值
	std::vector<double> cos_a;//用于存储对应维度的hough空间小格子中心点的cos值
	std::vector<double> sin_b;//用于存储对应维度的hough空间小格子中心点的sin值
	std::vector<double> cos_b;//用于存储对应维度的hough空间小格子中心点的cos值

	void ModifyPoint(const int& factor, const Coordinate& pt);//供AddPoint()和RemovePoint() 调用
	void ModifyPointAndDraw(const int& factor, const Coordinate& pt, std::vector<Coordinate>& pts_draw_once);
	int Maximum(const int& num_ignore, const Coordinate& window_size, int &i1max, int &i2max, int &i3max) ;//window_size的各维度坐标必须是整数, num_ignore表示忽略前几个最多的地方，如果num_ignore=0，则表示找点最多的地方
public:
	HoughTransform3D();
	HoughTransform3D(const Coordinate &dim_min, const Coordinate &dim_max, const Coordinate &dim_count, const Coordinate &offset, const HoughEquation &equation);
	HoughTransform3D(const std::vector<Coordinate>&pts, const Coordinate& resolution);
	HoughTransform3D(std::vector<std::vector<Coordinate> >& pts_draw, const std::vector<Coordinate>&pts, const Coordinate& resolution);
	void Initialise(const std::vector<Coordinate>&pts, const Coordinate& resolution);
	~HoughTransform3D();
	void Initialise();
	void Initialise(const Coordinate &dim_min, const Coordinate &dim_max, const Coordinate &dim_count, const Coordinate &offset, const HoughEquation &equation);
	void GetDimInfo(Coordinate &dim_min, Coordinate &dim_max, Coordinate &dim_count, Coordinate &offset, Coordinate &dim_resolution) const;
	void Print();
	void AddPoint(const Coordinate& pt);
	void RemovePoint(const Coordinate& pt);
	NaviPlane GetBestPlane(const int& num_ignore, const Coordinate& window_size, int &numpts);
};