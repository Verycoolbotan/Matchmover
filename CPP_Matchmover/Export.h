#pragma once
#include "Matchmover.h"

typedef struct quat
{
	double w;		//Скаляр
	double x, y, z; //Вектор
};

const char MAX_HEADER[] = "point cross:off centermarker:on pos:[0.0, 0.0, 0.0] name:\"origin\"\n\
point cross:off centermarker:on pos:[0.0, 0.0, 0.0] name:\"reference_points\" parent:$origin\n\
animate on(\n\
cam = freecamera name:\"Camera\"\n\
cam.nearclip = 0.0\n\
cam.parent = $origin\n\
cam.fov = %f\n";

const char MAX_CAM_TEMPLATE[] = "at time %df cam.rotation = quat %f %f %f %f\nat time %df cam.position = [%f, %f, %f]\n";
const char MAX_PT_TEMPLATE[] = "point cross:off centermarker:on pos:[%f, %f, %f] name:\"auto_%d\" parent:$reference_points\n";

quat matToQuat(Mat &R);
void toMax(vector<camera> &cameras, vector<sp_point> &points, string save_path);