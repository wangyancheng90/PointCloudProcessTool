#include "HoughTransform3D.h"
#include "Coordinate.h"
#include "WYCPlane.h"
#include "math.h"
#include<cstdlib>
#include <iostream>
#include <algorithm>
#include <float.h>
using namespace std;

void HoughTransform3D::Initialise()
{
	Coordinate zero(0.0, 0.0, 0.0);
	this->dim_min = zero;
	this->dim_max = zero;
	this->dim_count = zero;
	this->dim_resolution = zero;
	this->offset = zero;
	this->equation = PLANE_EQUATION_1;
	this->hough_space.clear();
	this->sin_a.clear();
	this->cos_a.clear();
	this->sin_b.clear();
	this->cos_b.clear();
}

void HoughTransform3D::Initialise(const Coordinate &dim_min1, const Coordinate &dim_max1, const Coordinate &dim_count1, const Coordinate &offset1, const HoughEquation &equation1)
{
	this->Initialise();
	this->dim_min = dim_min1;
	this->dim_max = dim_max1;
	this->dim_count = dim_count1;
	this->dim_resolution.x = (dim_max1.x - dim_min1.x ) / dim_count1.x;
	this->dim_resolution.y = (dim_max1.y - dim_min1.y) / dim_count1.y;
	this->dim_resolution.z = (dim_max1.z - dim_min1.z ) / dim_count1.z;
	this->offset = offset1;
	this->equation = equation1;
	vector<int> third_dim_space((int)dim_count1.z, 0);
	vector<vector<int> >second_third_dim_space;
	second_third_dim_space.resize((int)dim_count1.y);
	for (int i = 0; i < second_third_dim_space.size(); i++)
	{
		second_third_dim_space[i] = third_dim_space;
	}
	this->hough_space.resize((int)dim_count1.x);
	for (int i = 0; i < this->hough_space.size(); i++)
	{
		this->hough_space[i] = second_third_dim_space;
	}
	if (this->equation == PLANE_EQUATION_1)
	{
		this->sin_a.resize(dim_count1.x);
		this->cos_a.resize(dim_count1.x);
		double v = dim_min1.x + this->dim_resolution.x / 2.0;
		for (int i = 0; i < dim_count1.x; i++)
		{
			this->sin_a[i] = sin(v); this->cos_a[i] = cos(v);
			v += this->dim_resolution.x;
		}

		this->sin_b.resize(dim_count1.y);
		this->cos_b.resize(dim_count1.y);
		v = dim_min1.y + this->dim_resolution.y / 2.0;
		for (int i = 0 ; i < dim_count1.y;  i++)
		{
			this->sin_b[i] = sin(v); this->cos_b[i] = cos(v);
			v += this->dim_resolution.y;
		}
	}
}

void HoughTransform3D::Initialise(const vector<Coordinate>&pts,const Coordinate& resolution_orig)
{
	if (pts.size() < 3)
	{
		return;
	}
	Coordinate resolution(resolution_orig.x*(atan(1.0) / 45), resolution_orig.y*(atan(1.0) / 45), resolution_orig.z);
	Coordinate pts_max(-999999999, -999999999, -999999999 );
	Coordinate pts_min(DBL_MAX, DBL_MAX, DBL_MAX);
	Coordinate center(0.0, 0.0, 0.0);
	for (int i = 0; i < pts.size(); i++)
	{
		pts_min.x = (pts_min.x>pts[i].x) ? pts[i].x : pts_min.x;
		pts_min.y = (pts_min.y>pts[i].y) ? pts[i].y : pts_min.y;
		pts_min.z = (pts_min.z>pts[i].z) ? pts[i].z : pts_min.z;

		pts_max.x = (pts_max.x< pts[i].x) ? pts[i].x : pts_max.x;
		pts_max.y = (pts_max.y < pts[i].y) ? pts[i].y : pts_max.y;
		pts_max.z = (pts_max.z < pts[i].z) ? pts[i].z : pts_max.z;

		center.x += pts[i].x;
		center.y += pts[i].y;
		center.z += pts[i].z;
	}

	center = center / pts.size();
	Coordinate dim_min(0, 0 , -pts_max.Distance(pts_min) / 2);
	Coordinate dim_max(2*atan(1.0) , 8 * atan(1.0), pts_max.Distance(pts_min) / 2);
	Coordinate dim_count((int)((dim_max.x - dim_min.x) / resolution.x), (int)((dim_max.y-dim_min.y) / resolution.y), (int)(pts_max.Distance(pts_min) / resolution.z));
	Coordinate offset = (pts_max + pts_min) / 2;
	this->Initialise(dim_min, dim_max, dim_count, offset, PLANE_EQUATION_1);
	for (int i = 0; i < pts.size(); i++)
	{
		this->AddPoint(pts[i]);
	}
}

HoughTransform3D::HoughTransform3D()
{
	this->Initialise();
}

HoughTransform3D::HoughTransform3D(const Coordinate &dim_min1, const Coordinate &dim_max1, const Coordinate &dim_count1, const Coordinate &offset1, const HoughEquation &equation1)
{
	this->Initialise(dim_min1, dim_max1, dim_count1, offset1,equation1);
}

HoughTransform3D::HoughTransform3D(const vector<Coordinate>&pts, const Coordinate& resolution)
{
	this->Initialise(pts,resolution);
}

HoughTransform3D::HoughTransform3D(vector<vector<Coordinate> >& pts_draw,const std::vector<Coordinate>&pts, const Coordinate& resolution)
{
	this->Initialise(pts, resolution);
	pts_draw.clear();
	for (int i = 0; i < pts.size(); i++)
	{
		if (i % 100 == 0)
		{
			vector<Coordinate> pts_draw_once;
			this->ModifyPointAndDraw(1, pts[i], pts_draw_once);
			pts_draw.push_back(pts_draw_once);
		}
		else
		{
			this->AddPoint(pts[i]);
		}
	}

}


HoughTransform3D::~HoughTransform3D()
{
	NULL;
}

void HoughTransform3D::GetDimInfo(Coordinate &dim_min1, Coordinate &dim_max1, Coordinate &dim_count1, Coordinate &offset1, Coordinate &dim_resolution1) const
{
	dim_min1 = this->dim_min;
	dim_max1 = this->dim_max;
	dim_count1 = this->dim_count;
	dim_resolution1 = this->dim_resolution;
	dim_min1 = this->dim_min;
	offset1 = this->offset;
}

void HoughTransform3D::Print()
{
	cout << "HoughInfo:" << endl;
	cout << "min:	" << dim_min.x << " " << dim_min.y << " " << dim_min.z << " " << endl;
	cout << "max:	" << dim_max.x << " " << dim_max.y << " " << dim_max.z << " " << endl;
	cout << "count:		" << dim_count.x << " " << dim_count.y << " " << dim_count.z << " " << endl;
	cout << "resolution:	" << dim_resolution.x << " " << dim_resolution.y << " " << dim_resolution.z << " " << endl;
	cout << "offset:		" << offset.x << " " << offset.y << " " << offset.z << " " << endl;

}


void HoughTransform3D::ModifyPoint(const int& factor, const Coordinate& pt)//供AddPoint()和RemovePoint() 调用
{

	//没加入一个点，遍历第一维和第二维，求第三维的位置
	for (int i1 = 0; i1 < this->dim_count.x; i1++)
	{
		for (int i2 = 0; i2 < this->dim_count.y; i2++)
		{
			if (this->equation == PLANE_EQUATION_1)
			{
				double v3 = sin_a[i1] * (sin_b[i2] * (pt.x - this->offset.x) + cos_b[i2] * (pt.y - this->offset.y)) + cos_a[i1] * (pt.z - this->offset.z);
				int i3 = (int)((v3 - this->dim_min.z) / this->dim_resolution.z);
				if (i3 >= 0 && i3 < this->dim_count.z) this->hough_space[i1][i2][i3] += factor;
			}
		}
	}
}

void HoughTransform3D::ModifyPointAndDraw(const int& factor, const Coordinate& pt, vector<Coordinate>& pts_draw_once)
{
	int ran=rand() % 10000;
	pts_draw_once.clear();
	Coordinate pt_draw(-1,-1,-1);
	//没加入一个点，遍历第一维和第二维，求第三维的位置
	for (int i1 = 0; i1 < this->dim_count.x; i1++)
	{
		for (int i2 = 0; i2 < this->dim_count.y; i2++)
		{
			if (this->equation == PLANE_EQUATION_1)
			{
				double v3 = this->sin_a[i1] * (this->sin_b[i2] * (pt.x - this->offset.x) + this->cos_b[i2] * (pt.y - this->offset.y)) + this->cos_a[i1] * (pt.z - this->offset.z);
				int i3 = (int)((v3 - this->dim_min.z) / this->dim_resolution.z);
				if (i3 >= 0 && i3 < this->dim_count.z)
				{
					this->hough_space[i1][i2][i3] += factor;
					pt_draw.x = i1 + (((double)ran) - 5000) / 20000+0.5;
					pt_draw.y = i2 + (((double)ran) - 5000) / 20000 + 0.5;
					pt_draw.z = i3 + (((double)ran) - 5000) / 20000 + 0.5;
					pts_draw_once.push_back(pt_draw);
				}
			}
		}
	}
}

void HoughTransform3D::AddPoint(const Coordinate& pt)
{
	return this->ModifyPoint(1, pt);
}

void HoughTransform3D::RemovePoint(const Coordinate& pt)
{
	return this->ModifyPoint(-1, pt);
}

int HoughTransform3D::Maximum(const int& num_ignore, const Coordinate& influence_window_size, int &i1max, int &i2max, int &i3max)
{
	int numpts = -1;
	for (int ignored = 0; ignored <= num_ignore; ignored++)
	{
		// Find the next maximum
		numpts = -1;
		for (int i1 = 0; i1 < this->dim_count.x;i1++)
		{
			for (int i2 = 0; i2 < this->dim_count.y;i2++)
			{
				for (int i3 = 0; i3 < this->dim_count.z;i3++)
				{
					if (this->hough_space[i1][i2][i3] > numpts)
					{
						numpts = this->hough_space[i1][i2][i3];
						i1max = i1;  i2max = i2;  i3max = i3;
					}
				}
			}
		}

		// 如果这次不是需要找到的最后一次，那么将这次找到的结果格子置为负数，这样不干扰下次寻找
		if (ignored != num_ignore)
		{
			//如果是竖直平面，则存在第二维有重复，需要置负别的地方
			if (fabs((i1max*this->dim_resolution.x + 0.5*this->dim_resolution.x + this->dim_min.x) * 45 / atan(1.0) - 90) < 3)//竖直平面判断
			{
				//第二维+180度，第三维取对称地方
				for (int i1 = max<int>(0, i1max - influence_window_size.x / 2); i1 <= min<int>(this->dim_count.x - 1, i1max + influence_window_size.x / 2); i1++)
				{
					for (int i2 = max<int>(0, i2max + (4 * atan(1.0) / this->dim_resolution.y) - influence_window_size.y / 2); i2 <= min<int>(this->dim_count.y - 1, i2max + (4 * atan(1.0) / this->dim_resolution.y) + influence_window_size.y / 2); i2++)
					{
						for (int i3 = max<int>(0, (this->dim_count.z - i3max) - influence_window_size.z / 2); i3 <= min<int>(this->dim_count.z - 1, (this->dim_count.z - i3max) + influence_window_size.z / 2); i3++)
						{
							if (this->hough_space[i1][i2][i3] > 0)  this->hough_space[i1][i2][i3] = -1 * this->hough_space[i1][i2][i3];
						}
					}
				}
				//第二维-180度，第三维取对称地方
				for (int i1 = max<int>(0, i1max - influence_window_size.x / 2); i1 <= min<int>(this->dim_count.x - 1, i1max + influence_window_size.x / 2); i1++)
				{
					for (int i2 = max<int>(0, i2max - (4 * atan(1.0) / this->dim_resolution.y) - influence_window_size.y / 2); i2 <= min<int>(this->dim_count.y - 1, i2max - (4 * atan(1.0) / this->dim_resolution.y) + influence_window_size.y / 2); i2++)
					{
						for (int i3 = max<int>(0, (this->dim_count.z - i3max) - influence_window_size.z / 2); i3 <= min<int>(this->dim_count.z - 1, (this->dim_count.z - i3max) + influence_window_size.z / 2); i3++)
						{
							if (this->hough_space[i1][i2][i3] > 0)  this->hough_space[i1][i2][i3] = -1 * this->hough_space[i1][i2][i3];
						}
					}
				}
			}

			for (int i1 = max<int>(0, i1max - influence_window_size.x / 2); i1 <= min<int>(this->dim_count.x - 1, i1max + influence_window_size.x / 2); i1++)
			{
				for (int i2 = max<int>(0, i2max - influence_window_size.y / 2); i2 <= min<int>(this->dim_count.y - 1, i2max + influence_window_size.y / 2); i2++)
				{
					for (int i3 = max<int>(0, i3max - influence_window_size.z / 2); i3 <= min<int>(this->dim_count.z - 1, i3max + influence_window_size.z / 2); i3++)
					{
						if (this->hough_space[i1][i2][i3] > 0)  this->hough_space[i1][i2][i3] = -1 * this->hough_space[i1][i2][i3];
					}
				}
			}
		
				
		}
		// 将之前置为负数的格子，还原
		else if (num_ignore > 0 && ignored == num_ignore)
		{
			for (int i1 = 0 ; i1 < this->dim_count.x; i1++)
			{
				for (int i2 = 0; i2 < this->dim_count.y; i2++)
				{
					for (int i3 = 0; i3 < this->dim_count.z; i3++)
					{
						if (this->hough_space[i1][i2][i3] < 0)  this->hough_space[i1][i2][i3] = -1 * this->hough_space[i1][i2][i3];
					}
				}
			}
		}
	}
	return(numpts);
}

WYCPlane HoughTransform3D::GetBestPlane(const int& num_ignore, const Coordinate& influence_window_size, int &numpts)
{
	WYCPlane plane;
	int  i1max=-1, i2max=-1, i3max=-1;
	numpts = this->Maximum(num_ignore, influence_window_size, i1max, i2max, i3max);
	if (numpts > 0)
	{
		double v3 = this->dim_min.z + this->dim_resolution.z / 2.0 + i3max *  this->dim_resolution.z;
		if (this->equation == PLANE_EQUATION_1)
		{
			Coordinate normal(this->sin_a[i1max] * this->sin_b[i2max], this->sin_a[i1max] * this->cos_b[i2max], this->cos_a[i1max]);
			normal.Normalize();
			double dist = v3 +this->offset.x * this->sin_a[i1max] * this->sin_b[i2max] +this->offset.y * this->sin_a[i1max] * this->cos_b[i2max] +this->offset.z * this->cos_a[i1max];
			return WYCPlane(dist, normal);
		}
		else
		{
			return plane;
		}
	}
	else
	{
		return plane;
	}
}

