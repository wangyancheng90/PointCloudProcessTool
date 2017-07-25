#include "Coordinate.h"
#include "math.h"

Coordinate::Coordinate()
{
	{
		this->x = 0;
		this->y = 0;
		this->z = 0;
	}
}

Coordinate::Coordinate(double x1, double y1, double z1)
{
	this->x = x1;
	this->y = y1;
	this->z = z1;
}

//复制构造函数,深拷贝
Coordinate::Coordinate(const Coordinate &rhs)
{
	this->x = rhs.x;
	this->y = rhs.y;
	this->z = rhs.z;
}

Coordinate::~Coordinate()
{

}

//赋值操作符,深拷贝
Coordinate& Coordinate::operator = (const Coordinate &rhs)
{
	this->x = rhs.x;
	this->y = rhs.y;
	this->z = rhs.z;
	return *this;
	//return Coordinate(rhs.x, rhs.y,rhs.z);
}

///计算两个坐标之间的距离
double Coordinate::DistanceXY(const Coordinate& p) const
{
	double dx = x - p.x;
	double dy = y - p.y;
	return sqrt(dx * dx + dy * dy);
}


///计算两个坐标之间的距离
double Coordinate::DistanceXYSQ(const Coordinate& p) const
{
	double dx = x - p.x;
	double dy = y - p.y;
	return dx * dx + dy * dy;
}


double Coordinate::Distance(const Coordinate& p) const
{
	double dx = x - p.x;
	double dy = y - p.y;
	double dz = z - p.z;
	return sqrt(dx * dx + dy * dy + dz * dz);
}

double Coordinate::DistanceSQ(const Coordinate& p) const
{
	double dx = x - p.x;
	double dy = y - p.y;
	double dz = z - p.z;
	return dx * dx + dy * dy + dz * dz;
}

double Coordinate::GetLength() const
{
	return sqrt(x*x + y*y + z*z);
}

void Coordinate::Normalize()
{
	double lenth = GetLength();

	if (lenth <= 1e-5)
	{
		return;
	}

	x = x / lenth;
	y = y / lenth;
	z = z / lenth;
}

//重载运算符-
Coordinate Coordinate::operator -(const Coordinate &rhs) const
{
	return Coordinate(x - rhs.x, y - rhs.y, z - rhs.z);
}

//重载运算符+
Coordinate Coordinate::operator +(const Coordinate &rhs) const
{
	return Coordinate(x + rhs.x, y + rhs.y, z + rhs.z);
}

//重载运算符*,将坐标看做向量运算
double Coordinate::operator *(const Coordinate &rhs) const
{
	return x*rhs.x + y*rhs.y + z*rhs.z;
}

Coordinate Coordinate::operator *(double N) const
{
	return Coordinate(x*N, y*N, z*N);
}

Coordinate Coordinate::operator /(double N) const
{
	return Coordinate(x / N, y / N, z / N);
}

Coordinate Coordinate::CrossProduct(const Coordinate& other) const
{
	double ax = y*other.z - z*other.y;
	double ay = z*other.x - x*other.z;
	double az = x*other.y - y*other.x;
	return Coordinate(ax, ay, az);
}

double Coordinate::DotProduct(const Coordinate &pnt) const
{
	return x * pnt.x + y * pnt.y + z * pnt.z;
}

double Coordinate::DotProductXY(const Coordinate &pnt) const
{
	return x * pnt.x + y * pnt.y;
}

bool projPointCmp(const projPoint c0, const projPoint c1)
{
	return c0.project < c1.project;
}