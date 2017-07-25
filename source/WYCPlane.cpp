#include <stdio.h>
#include <iostream>
#include <Eigen/Eigenvalues> 
#include <ctime>
#include "WYCPlane.h"
#include "Coordinate.h"
using namespace std;

void WYCPlane::Initialise()
{
	Coordinate zero(0.0, 0.0, 0.0);
	this->normal = zero;
	this->offset = zero;
	this->coord_sum = zero;
	this->distance = 0;
	this->num_pts = 0;
	this->label = 0;
	this->segment_number = 0;
	this->process_tag = 0;
	this->smallest_eigenvalue = 0;
	vector<double> zeros_3(3, 0.0);
	this->moments.resize(3);
	for (int i = 0; i < 3; i++)
	{
		this->moments[i] = zeros_3;
	}
}

WYCPlane::WYCPlane()
{
	this->Initialise();
}

WYCPlane::~WYCPlane()
{
	NULL;
}

void WYCPlane::CheckNormal()
{
	if (this->normal.z == 0 && this->normal.y == 0)
	{
		if (this->normal.x < 0)
		{
			this->normal.x *= -1;
		}
	}
	else if (this->normal.z == 0 && this->normal.x == 0)
	{
		if (this->normal.y< 0)
		{
			this->normal.y *= -1;
		}
	}
	else if (this->normal.x == 0 && this->normal.y == 0)
	{
		if (this->normal.z < 0)
		{
			this->normal.z *= -1;
		}
	}
	else if (this->normal.z == 0)
	{
		if (this->normal.x < 0)
		{
			this->normal.x *= -1;
			this->normal.y *= -1;
		}
	}
	else if (this->normal.y == 0)
	{
		if (this->normal.z < 0)
		{
			this->normal.x *= -1;
			this->normal.z *= -1;
		}
	}
	else if (this->normal.x == 0)
	{
		if (this->normal.z < 0)
		{
			this->normal.y *= -1;
			this->normal.z *= -1;
		}
	}
	else if (this->normal.z< 0)
	{
		this->normal.x *= -1;
		this->normal.y *= -1;
		this->normal.z *= -1;
	}
}

WYCPlane::WYCPlane(const Coordinate&pt, const Coordinate&normal1)
{
	this->Initialise();
	double len = normal1.GetLength();
	if (len == 0.0)
	{
		this->normal = Coordinate(0.0, 0.0, 1.0);
		this->distance = pt.z;
	}
	else
	{
		this->normal = normal1;
		this->normal.Normalize();
		this->CheckNormal();
		this->distance = this->normal.DotProduct(pt);
	}
}

WYCPlane::WYCPlane(const double&dist, const Coordinate&normal1)
{
	this->Initialise();
	this->normal = normal1;
	this->normal.Normalize();
	this->CheckNormal();
	this->distance = dist;
}

void WYCPlane::SetOffset(const Coordinate& offset1)///// Offset for moment computations
{
	this->offset = offset1;
}

void WYCPlane::Print() const
{
	cout << "Plane[" << num_pts << "], " << normal.x << " " << normal.y << " " << normal.z << "; " << distance << "; " << offset.x << " " << offset.y << " " << offset.z << endl;
}

WYCPlane& WYCPlane::operator=(const WYCPlane& pl)
{
	this->Initialise();
	this->normal = pl.normal;
	this->offset = pl.offset;
	this->coord_sum = pl.coord_sum;
	this->distance = pl.distance;
	this->num_pts = pl.num_pts;
	this->segment_number = pl.segment_number;
	this->label = pl.label;
	this->smallest_eigenvalue = pl.smallest_eigenvalue;
	this->process_tag = pl.process_tag;
	this->moments = pl.moments;

	return *this;
}

double  WYCPlane::Distance(const Coordinate& point) const //可正可负
{
	return(point.x * this->normal.x + point.y * this->normal.y + point.z *this->normal.z - this->distance);
}

void WYCPlane::Projection(const Coordinate &pt_orig, Coordinate &pt_dest) const
{
	double  dist_point = this->Distance(pt_orig);
	pt_dest.x = pt_orig.x - dist_point * this->normal.x;
	pt_dest.y = pt_orig.y - dist_point * this->normal.y;
	pt_dest.z = pt_orig.z - dist_point * this->normal.z;
}

bool WYCPlane::Recalculate()
{
	if (this->num_pts < 3) return false;

	//生成协方差矩阵
	Eigen::Matrix3d central_moments = Eigen::MatrixXd::Zero(3, 3);
	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j <=3; j++)
		{
			central_moments(i, j) = this->moments[i][j];
		}
	}
	Coordinate centre_coord = this->coord_sum / (double)num_pts;
	vector<double> center_vect(3, 0.0);
	center_vect[0] = centre_coord.x;
	center_vect[1] = centre_coord.y;
	center_vect[2] = centre_coord.z;
	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j <= i; j++)
		{
			central_moments(i,j) = this->moments[i][j]- this->num_pts * center_vect[i] * center_vect[j];
		}
	}
	// 镜像填满协方差矩阵
	central_moments(0,1) = central_moments(1,0);
	central_moments(0,2) = central_moments(2,0);
	central_moments(1,2) = central_moments(2,1);
	//算特征值
	Eigen::EigenSolver<Eigen::Matrix3d> es(central_moments);
	vector<double> eigen_valuses(3, 0.0);
	eigen_valuses[0] = es.eigenvalues()[0].real();
	eigen_valuses[1] = es.eigenvalues()[1].real();
	eigen_valuses[2] = es.eigenvalues()[2].real();

	// Select the smallest eigen value
	int smallest_index = 0;
	smallest_index = (eigen_valuses[0] < eigen_valuses[1]) ? 0 : 1;
	smallest_index = (eigen_valuses[smallest_index] < eigen_valuses[2]) ? smallest_index : 2;
	this->smallest_eigenvalue = eigen_valuses[smallest_index];

	this->normal.x = es.eigenvectors().col(smallest_index)(0).real();
	this->normal.y = es.eigenvectors().col(smallest_index)(1).real();
	this->normal.z = es.eigenvectors().col(smallest_index)(2).real();
	this->normal.Normalize();
	this->CheckNormal();
	this->distance = this->normal.DotProduct(centre_coord + this->offset);
	return true;
}

bool WYCPlane::ChangePoint(const Coordinate &pos, const bool& add, const bool& recalculate)
{
	if (this->num_pts == 0 && this->offset.x == 0.0 && this->offset.y == 0.0 && this->offset.z == 0.0)  this->offset = pos;
	Coordinate diff(0.0,0.0,0.0);
	diff = pos - this->offset;
	vector<double> diff_vect(3, 0.0);
	diff_vect[0] = diff.x;
	diff_vect[1] = diff.y;
	diff_vect[2] = diff.z;
	if (add)
	{
		this->num_pts++;              // 更新num, sum，moment
		this->coord_sum = this->coord_sum+diff;
		for (int i = 0; i < 3; i++)
		{
			for (int j = 0; j <= i; j++)
			{
				this->moments[i][j] += diff_vect[i] * diff_vect[j];
			}
		}
	}
	else
	{
		this->num_pts--;               // 更新num, sum，moment
		this->coord_sum = this->coord_sum - diff;
		for (int i = 0; i < 3; i++)
		{
			for (int j = 0; j <= i; j++)
			{
				this->moments[i][j] -= diff_vect[i] * diff_vect[j];
			}
		}
	}
	if (recalculate) return this->Recalculate();
	else return false;
}

bool WYCPlane::AddPoint(const Coordinate &pos, const bool& recalculate)
{
	return ChangePoint(pos, true, recalculate);
}

void WYCPlane::Initialise(const std::vector<Coordinate>& points)
{
	this->Initialise();
	for (int i = 0; i < points.size(); i++)
	{
		if (i != points.size() - 1) 		this->AddPoint(points[i], false);
		else this->AddPoint(points[i], true);
	}
}


WYCPlane::WYCPlane(const vector<Coordinate>& points)
{
	this->Initialise(points);
}


bool WYCPlane::RemovePoint(const Coordinate &pos, const bool& recalculate)
{
	return ChangePoint(pos, false, recalculate);
}

void WYCPlane::SetLabel(const int& value)
{
	this->label = value;
}
void WYCPlane::SetSegmentNumber(const int& value)
{
	this->segment_number = value;
}
void WYCPlane::SetProcessTag(const int& value)
{
	this->process_tag = value;
}

int WYCPlane::GetLabel()
{
	return this->label;
}
int WYCPlane::GetSegmetNumber()
{
	return this->segment_number;
}
int WYCPlane::GetProcessTag()
{
	return this->process_tag;
}

Coordinate WYCPlane::GetPlaneNormal()
{
	return this->normal;
}

double WYCPlane::GetPlaneDistance()
{
	return this->distance;
}

int WYCPlane::GetPointNumber()
{
	return this->num_pts;
}
Timer::Timer()
{
	b_time = time(0);
}
Timer::~Timer()
{
	cout << "TIME: " << time(0) - b_time << endl;
}