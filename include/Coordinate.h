#pragma once
#include <stdio.h>  
#include <stdlib.h>  
#include <memory>  
#include "stdlib.h"  
#include "string.h"  
#include <sstream>
#include <fstream>
#include <vector>

class  Coordinate
{
public:
	//默认构造函数,x,y,z都等于0
	Coordinate();
	//使用给定的x,y,z构造Coordinate对象
	Coordinate(double x1, double y1, double z1);

	//复制构造函数,深拷贝
	Coordinate(const Coordinate &rhs);
	//析构函数
	~Coordinate();

	//赋值操作符,深拷贝
	Coordinate &operator = (const Coordinate &rhs);
	//重载运算符-
	Coordinate operator -(const Coordinate &rhs) const;

	//重载运算符+
	Coordinate operator +(const Coordinate &rhs) const;

	//重载运算符*,将坐标看做向量运算
	double operator *(const Coordinate &rhs) const;

	//重载运算符*,x,y,z均放大N倍
	Coordinate operator *(double N) const;

	//叉乘
	Coordinate CrossProduct(const Coordinate& other) const;

	//点乘
	double DotProduct(const Coordinate &pnt) const;

	//点乘XY方向
	double DotProductXY(const Coordinate &pnt) const;

	//重载运算符/,x,y,z均缩小N倍
	Coordinate operator /(double N) const;

	//将坐标看做向量,求向量长度
	double GetLength() const;

	//将坐标看做向量进行单位化
	void Normalize();

	// XY方向距离 [ZOSH 2017/06/06]
	double DistanceXY(const Coordinate& p) const;

	// XY方向距离平方 [ZOSH 2017/06/06]
	double DistanceXYSQ(const Coordinate& p) const;

	// 距离 [ZOSH 2017/06/06]
	double Distance(const Coordinate& p) const;

	// 距离的平方 [ZOSH 2017/06/06]
	double DistanceSQ(const Coordinate& p) const;

public:
	double x;
	double y;
	double z;
};

struct projPoint
{
	double project;
	Coordinate c;
} ;

bool projPointCmp(const projPoint c0, const projPoint c1);
