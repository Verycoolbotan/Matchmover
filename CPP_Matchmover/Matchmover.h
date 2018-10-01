#pragma once
#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>
#include <map>

using namespace cv;
using namespace std;

typedef struct camera
{
	Mat image;
	Mat P;		//Матрица проекции (3*4)
	Mat T;		//Вращение и смещение (4*4)
	Mat des;	//Дескрипторы для найденных точек
	vector<KeyPoint> kp; //Найденные точки
	//TODO: разберись с беззнаковыми типами
	map<int, map<int, int> > matches; //Найденные соответствия в виде {точка:{камера:точка}}
	map<int, int> matches3d; //{точка на изображении:точка в пространстве}
};

typedef struct sp_point
{
	Point3d pt;		//Координаты точки в пространстве
	int seen = 2;	//Сколько камер её увидели
};

extern double fov; //Угол зрения камеры для экспорта

extern vector<camera> camList;		//Список камер
extern vector<sp_point> points3D;	//Список восстановленных точек

void initialize(vector<string> &pathList);
void matchFeatures();
void reconstruct(double focal, double W, double H);