/****************************************************************************************************************************************
File name: WYCPlane.h
Author: Yancheng Wang
Version: 1.0
Date: 2017-7-15
Description: A plane class could quick dynamic adjustment
****************************************************************************************************************************************/
#pragma once
#include "Coordinate.h"
class Timer
{
public:
	time_t b_time;
	Timer();
	~Timer();
};

class WYCPlane
{
protected:
	Coordinate normal;
	Coordinate offset;
	Coordinate coord_sum;
	double distance;//到原点的距离
	int num_pts;
	int label;
	int segment_number;
	int process_tag;
	double smallest_eigenvalue;
	std::vector<std::vector<double> >moments;//3*3 矩
	void CheckNormal();
	bool ChangePoint(const Coordinate &pos, const bool& add, const bool& recalculate = true);
public:
	WYCPlane();
	WYCPlane(const Coordinate&pt, const Coordinate&normal);
	WYCPlane(const double&dist, const Coordinate&normal);
	WYCPlane(const std::vector<Coordinate>& points);
	void SetOffset(const Coordinate& offset);

	~WYCPlane();
	void Initialise();
	void Initialise(const std::vector<Coordinate>& points);
	WYCPlane& operator=(const WYCPlane& pl);
	void Print() const;
	bool AddPoint(const Coordinate &pos, const bool& recalculate = true);
	bool RemovePoint(const Coordinate &pos, const bool& recalculate = true);
	double Distance(const Coordinate& pt) const;
	void Projection(const Coordinate &pt_orig, Coordinate &pt_dest) const;
	void SetLabel(const int& value);
	void SetSegmentNumber(const int& value);
	void SetProcessTag(const int& value);
	int GetLabel();
	int GetSegmetNumber();
	int GetProcessTag();
	Coordinate GetPlaneNormal();
	double GetPlaneDistance();
	int GetPointNumber();
	bool Recalculate();
};