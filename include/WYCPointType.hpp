/****************************************************************************************************************************************
File name: WYCPointType.hpp
Author: Yancheng Wang
Version: 1.0
Date: 2016-10-15
Description: 定义点云的点类型
****************************************************************************************************************************************/
#pragma  once
#define PCL_NO_PRECOMPILE
#include <pcl/point_types.h>
#include <pcl/register_point_struct.h>
using namespace std;


struct _WYC_POINT_TYPE
{
	PCL_ADD_POINT4D;    // float data[4]
	PCL_ADD_NORMAL4D;//float data_n[4]
	PCL_ADD_RGB;//float data_c[4] rgb值被压缩到一个float数里，也就是data_c[1],data_c[2],data_c[3]都是空的

	struct
	{
		double GPSTime;//8
		unsigned short Intensity;//2
	};


	struct
	{
		int LabelTag;//4
		int SegmentNumberTag;//4
		int UserIntTag1;//4
		float Curvature;//4
	};


	struct
	{
		double UserDoubleTag1;//8
		double UserDoubleTag2;//8
	};


	struct
	{
		unsigned short PointSourceID;//2~
		uint8_t  ScanFlags;//1
		uint8_t  Classification;//1
		int8_t    ScanAngleRank;//1
		uint8_t   UserData;//1

	};


	EIGEN_MAKE_ALIGNED_OPERATOR_NEW   // make sure our new allocators are aligned 按16字节对齐
} EIGEN_ALIGN16;                      // enforce SSE padding for correct memory alignment

struct WYC_POINT_TYPE : public _WYC_POINT_TYPE
{
	inline WYC_POINT_TYPE(const _WYC_POINT_TYPE &p)
	{
		x = p.x;
		y = p.y;
		z = p.z;
		data[3] = 1.0f;

		normal_x = p.normal_x;
		normal_y = p.normal_y;
		normal_z = p.normal_z;
		data_n[3] = 0.0f;

		rgb = p.rgb;

		GPSTime = p.GPSTime;
		Intensity = p.Intensity;

		LabelTag = p.LabelTag;
		SegmentNumberTag = p.SegmentNumberTag;
		UserIntTag1 = p.UserIntTag1;
		Curvature = p.Curvature;

		UserDoubleTag1 = p.UserDoubleTag1;
		UserDoubleTag2 = p.UserDoubleTag2;

		PointSourceID = p.PointSourceID;
		ScanFlags = p.ScanFlags;
		Classification = p.Classification;
		ScanAngleRank = p.ScanAngleRank;
		UserData = p.UserData;
	}
	inline WYC_POINT_TYPE()
	{
		x = 0.0f;
		y = 0.0f;
		z = 0.0f;
		data[3] = 1.0f;

		normal_x = 0.0f;
		normal_y = 0.0f;
		normal_z = 0.0f;
		data_n[3] = 0.0f;

		r = 0;
		g = 0;
		b = 0;

		GPSTime = 0.0;
		Intensity = 0;

		LabelTag = 0;
		SegmentNumberTag = 0;
		UserIntTag1 = 0;
		Curvature = 0.0;

		UserDoubleTag1 = 0.0;
		UserDoubleTag2 = 0.0;

		PointSourceID = 0;//unsign short,
		ScanFlags = 0;//unsigned char
		Classification = 0;//unsigned char
		ScanAngleRank = 0;//char
		UserData = 0;//signed char



	}
	inline WYC_POINT_TYPE(float X, float Y, float Z)
	{
		x = X;
		y = Y;
		z = Z;
		data[3] = 1.0f;
		normal_x = 0.0f;
		normal_y = 0.0f;
		normal_z = 0.0f;
		data_n[3] = 0.0f;

		r = 0;
		g = 0;
		b = 0;

		GPSTime = 0.0;
		Intensity = 0;

		LabelTag = 0;
		SegmentNumberTag = 0;
		UserIntTag1 = 0;
		Curvature = 0.0;

		UserDoubleTag1 = 0.0;
		UserDoubleTag2 = 0.0;

		PointSourceID = 0;//unsign short,
		ScanFlags = 0;//unsigned char
		Classification = 0;//unsigned char
		ScanAngleRank = 0;//char
		UserData = 0;//signed char
	}

};
POINT_CLOUD_REGISTER_POINT_WRAPPER(WYC_POINT_TYPE, _WYC_POINT_TYPE)

POINT_CLOUD_REGISTER_POINT_STRUCT(WYC_POINT_TYPE,
(float, x, x)
(float, y, y)
(float, z, z)
(float, normal_x, normal_x)
(float, normal_y, normal_y)
(float, normal_z, normal_z)
(float, rgb, rgb)
(unsigned short, Intensity, Intensity)
(double, GPSTime, GPSTime)
(int, LabelTag, LabelTag)
(int, SegmentNumberTag, SegmentNumberTag)
(int, UserIntTag1, UserIntTag1)
(float, Curvature, Curvature)
(double, UserDoubleTag1, UserDoubleTag1)
(double, UserDoubleTag2, UserDoubleTag2)
(unsigned short, PointSourceID, PointSourceID)
(uint8_t, ScanFlags, ScanFlags)
(uint8_t, Classification, Classification)
(int8_t, ScanAngleRank, ScanAngleRank)
(uint8_t, UserData, UserData)
)

enum PointAttributeTag//int=1, float=2
{
	LAB_TAG = 1, SEGMENT_NUMBER_TAG = 2, USER_INT_TAG1 = 3, USER_DOUBLE_TAG1 = 4, USER_DOUBLE_TAG2 = 5,USER_DATA=6, CLASSIFICATION_TAG=7
};
typedef WYC_POINT_TYPE PointT;

