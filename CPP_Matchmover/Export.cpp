#include "Export.h"
#include <fstream>
#include <iostream>

quat matToQuat(Mat & R)
{
	quat q;
	double tr, s, qa[4];
	int i, j, k;

	int nxt[3] = { 1, 2, 0 };

	tr = R.at<double>(0, 0) + R.at<double>(1, 1) + R.at<double>(2, 2);

	if (tr > 0.0)
	{
		s = sqrt(tr + 1.0);
		q.w = s / 2.0;
		s = 0.5 / s;
		q.x = (R.at<double>(1, 2) - R.at<double>(2, 1)) * s;
		q.y = (R.at<double>(2, 0) - R.at<double>(0, 2)) * s;
		q.z = (R.at<double>(0, 1) - R.at<double>(1, 0)) * s;
	}
	else {
		i = 0;
		if (R.at<double>(1, 1) > R.at<double>(0, 0)) i = 1;
		if (R.at<double>(2, 2) > R.at<double>(i, i)) i = 2;
		j = nxt[i];
		k = nxt[j];

		s = sqrt(R.at<double>(i, i) - (R.at<double>(j, j) + R.at<double>(k, k)) + 1.0);

		qa[i] = s * 0.5;

		//Нельзя просто так взять и сравнить два double. Добавить машинный ноль?
		if (s != 0.0) s = 0.5 / s;

		qa[3] = (R.at<double>(j, k) - R.at<double>(k, j)) * s;
		qa[j] = (R.at<double>(i, j) - R.at<double>(j, i)) * s;
		qa[k] = (R.at<double>(i, k) - R.at<double>(k, i)) * s;

		q.x = qa[0];
		q.y = qa[1];
		q.z = qa[2];
		q.w = qa[3];
	}
	
	cout << "Success" << endl;

	return q;
}

void toMax(vector<camera>& cameras, vector<sp_point>& points, string save_path)
{
	ofstream out;
	out.open(save_path);

	char buff[256];
	int code = sprintf(buff, MAX_HEADER, fov);
	string str(buff);

	out << str;

	for (int i = 0; i < cameras.size(); i++)
	{
		Mat T = cameras[i].T;
		quat q = matToQuat(T);
		int code = sprintf(buff, MAX_CAM_TEMPLATE, i, q.w, q.x, q.y, q.z, i, T.at<double>(0, 3), T.at<double>(1, 3), T.at<double>(2, 3));
		string str(buff);
		out << str;
	}
	out << ")\n";

	for (int i = 0; i < points.size(); i++)
	{
		int code = sprintf(buff, MAX_PT_TEMPLATE, points[i].pt.x, points[i].pt.y, points[i].pt.z, i);
		string str(buff);
		out << str;
	}

	out.close();
}
