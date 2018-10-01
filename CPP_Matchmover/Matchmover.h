#pragma once
#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>
#include <map>

using namespace cv;
using namespace std;

typedef struct camera
{
	Mat image;
	Mat P;		//������� �������� (3*4)
	Mat T;		//�������� � �������� (4*4)
	Mat des;	//����������� ��� ��������� �����
	vector<KeyPoint> kp; //��������� �����
	//TODO: ��������� � ������������ ������
	map<int, map<int, int> > matches; //��������� ������������ � ���� {�����:{������:�����}}
	map<int, int> matches3d; //{����� �� �����������:����� � ������������}
};

typedef struct sp_point
{
	Point3d pt;		//���������� ����� � ������������
	int seen = 2;	//������� ����� � �������
};

extern double fov; //���� ������ ������ ��� ��������

extern vector<camera> camList;		//������ �����
extern vector<sp_point> points3D;	//������ ��������������� �����

void initialize(vector<string> &pathList);
void matchFeatures();
void reconstruct(double focal, double W, double H);