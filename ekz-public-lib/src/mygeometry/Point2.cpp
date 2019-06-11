#include "Point2.h"
Point2::Point2(float x_, float y_, float z_, float w_,float h_)
{
	x 			= x_;
	y 			= y_;
	z 			= z_;
	w 			= w_;
	h 			= h_;
	pos(0) 		= x;
	pos(1) 		= y;
	pos(2) 		= z;
	keypoint	= 0;
}
Point2::Point2()
{
	x 			= 0;
	y 			= 0;
	z 			= 0;
	w 			= 0;
	h 			= 0;
	pos(0) 		= 0;
	pos(1) 		= 0;
	pos(2) 		= 0;
	keypoint	= 0;
}
Point2::~Point2()
{
}

float Point2::distance(Point2 * p)
{
	float dx = x - p->x;
	float dy = y - p->y;
	float dz = z - p->z;
	return sqrt(dx*dx+dy*dy+dz*dz);
}

void Point2::print()
{
	//printf("Point2 w:%i,h:%i,x:%f,y:%f,z:%f\n",w,h,x,y,z);
}
