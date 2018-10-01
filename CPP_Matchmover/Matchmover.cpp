#include "Matchmover.h"
#include <opencv2/imgcodecs.hpp>
#include <opencv2/calib3d.hpp>
#include <iostream>
#include <ctime>
#include <cmath>


void initialize(vector<string> &pathList)
{
	cout << "Initializing..." << endl;

	int counter = 1;
	int size = pathList.size();

	//TODO: ����������� � ������ �����������
	Ptr<ORB> detector = ORB::create();

	for (string path : pathList)
	{
		cout << format("Processing image %d of %d", counter, size) << endl;
		camera cam;
		//TODO: ������ �� ������
		cam.image = imread(path);
		detector->detectAndCompute(cam.image, noArray(), cam.kp, cam.des);
		camList.push_back(cam);
		counter++;
	}
}

void matchFeatures()
{
	/*TODO: ��������� ���� ������� ���������������, ��������� � new
	 *���������� ��� ��������� ������ � ����*/
	BFMatcher matcher(NORM_HAMMING, true);

	/*� ����� ����������, ��� ����������� ������������ �� �������� "������ � ������".
	 *��� *�������* ���������, ��� ����� ���������� ���� ��������� ������*/
	for (int i = 0; i < camList.size() - 1; i++)
	{
		for (int j = i + 1; j < camList.size(); j++)
		{
			cout << format("Matching features; pair %d, %d", i, j) << endl;
			vector<DMatch> matches;
			matcher.match(camList[i].des, camList[j].des, matches);

			vector<Point2d> source, destination;	//���������� ��� �������������� ����������
			vector<int> i_kp, j_kp;	//������� ����� ��� ������ � map

			for (DMatch m : matches)
			{
				source.push_back(camList[i].kp[m.queryIdx].pt);
				destination.push_back(camList[j].kp[m.trainIdx].pt);
				i_kp.push_back(m.queryIdx);	//query - ��� ����
				j_kp.push_back(m.trainIdx); //train - ��� ����
			}

			/*crossCheck � ���������� matcher ��� ������ ������� ����� ������ ������������, ��
			 *����� �������������� ���������� �� ����������� ������������*/
			vector<uchar> mask;
			Mat F = findFundamentalMat(source, destination, FM_RANSAC, 3, 0.99, mask);

			//����� ������������ � map
			for (int k = 0; k < mask.size(); k++)
			{
				//���� ������� ������� != 0, �� ����� � �������� k ������������� ������������
				if (mask[k])
				{
					camList[i].matches[i_kp[k]][j] = j_kp[k];
					camList[j].matches[j_kp[k]][i] = i_kp[k];
				}
			}
		}
	}
}

void reconstruct(double focal, double W, double H)
{
	cout << "Reconstructing..." << endl;
	/*���� ���� ����� ������ ����� ������� � ���, � ������ ����� ���������� ���������� ����������� (������� Nghia Ho).
	 *������� ����� ������������ �������, ��� ������ ���� ����� �� ������������ (����. essential) ������� ���������
	 *������������� �������� � �������� (Hartley, Zisserman, Multiple View Geometry in Computer Vision, ������ �������,
	 *���. 258; https://courses.graphicon.ru/files/courses/vision/2010/cv_2010_13.pdf).
	 *������������ �������������� - ������ ����������� ������� ���������� � ������� ������. �� ����� ������������.
	 *C������� �������������, �.�. �������� ������ ��� �����������.
	 *�������� ������������ ��� ������ ���� (1 � 2), � ����� ��� ������ (2 � 3). ��������� ��������� ������� ������������,
	 *������� ��������������� ���� � ������ ������, ���� ����������. ��������� ���� ���������� - ������� �����������
	 *���������������.*/


	 /* focal - �������� ���������� � ��
	  * W, H - ���������� ������� ������� � ��
	  * distortion - ������������ ���������, �� ��������� �������*/

	srand(time(0));
	fov = atan((W / 2) / focal) / 3.14159265 * 180;

	//�������� ���������� � �������� (������������� ������)
	double fx = focal * camList[0].image.cols / W;
	double fy = focal * camList[0].image.rows / H;
	//���������� ������, �� �� principal point
	double cx = camList[0].image.cols / 2;
	double cy = camList[0].image.rows / 2;

	//�������� ������� ���������� ����������
	Mat K = Mat::eye(3, 3, CV_64F);
	K.at<double>(0, 0) = fx;
	K.at<double>(1, 1) = fy;
	K.at<double>(0, 2) = cx;
	K.at<double>(1, 2) = cy;

	camList[0].T = Mat::eye(4, 4, CV_64F);
	camList[0].P = K * Mat::eye(3, 4, CV_64F);

	//��������������� �������� ����� ������ �������� ������
	for (int i = 0; i < camList.size() - 1; i++)
	{
		camera& prev = camList[i];
		camera& curr = camList[i + 1];
		vector<Point2d> source, destination;
		vector<int> used;

		for (int j = 0; j < prev.kp.size(); j++)
		{
			if (prev.matches[j][i + 1]) //���� ���������� ������������ ����� j �� i+1-� �����
			{
				int index = prev.matches[j][i + 1];
				//�� ���������� �
				source.push_back(prev.kp[j].pt);
				destination.push_back(curr.kp[index].pt);
				used.push_back(j);
			}

		}

		cout << "Finding E..." << endl;
		Mat mask;
		Mat E = findEssentialMat(destination, source, K, RANSAC, 0.999, 1.0, mask);
		Mat local_R, local_t;
		recoverPose(E, destination, source, K, local_R, local_t);

		Mat T = Mat::eye(4, 4, CV_64F);
		local_R.copyTo(T(Range(0, 3), Range(0, 3)));
		//������ ������ ��� ����� � �������� ������
		local_t.copyTo(T(Range(0, 3), Range(3, 4)));
		//���� ����� ����������� ���������
		curr.T = prev.T * T; //����������� ��������������

		Mat P(3, 4, CV_64F);
		Mat R = curr.T(Range(0, 3), Range(0, 3)).clone();
		Mat t = curr.T(Range(0, 3), Range(3, 4)).clone();
		//0 = RC + t, C = -R.t() * t, ��� C - ����� ������
		((Mat)(-R.t() * t)).copyTo(P(Range(0, 3), Range(3, 4)));
		((Mat)(R.t())).copyTo(P(Range(0, 3), Range(0, 3)));
		curr.P = K * P; //����� ������� ��������

		Mat points4D;
		triangulatePoints(prev.P, curr.P, source, destination, points4D);

		/*������������ �� ���, ����� ��������������� ����� �������.
		 *���������� ��������� ���������� ����� ������� ������� */
		if (i > 0)
		{
			double scale = 0;
			int count = 0;

			Point3d prev_cam;
			prev_cam.x = prev.T.at<double>(0, 3);
			prev_cam.y = prev.T.at<double>(1, 3);
			prev_cam.z = prev.T.at<double>(2, 3);

			vector<Point3d> new_p, existing_p;

			for (int j = 0; j < used.size(); j++)
			{
				//������� ���� ��������������� �����, ����� �������� ����� ������ ����������
				int k = used[j];

				if (mask.at<uchar>(j) && prev.matches[k].count(i + 1) > 0 && prev.matches3d.count(k) > 0)
				{
					//��� �����, ��� ������������ cv::convertPointsFromHomogeneous
					Point3d pt;
					pt.x = points4D.at<double>(0, j) / points4D.at<double>(3, j);
					pt.y = points4D.at<double>(1, j) / points4D.at<double>(3, j);
					pt.z = points4D.at<double>(2, j) / points4D.at<double>(3, j);

					int index = prev.matches3d[k];
					Point3d average = points3D[index].pt / (points3D[index].seen - 1);

					new_p.push_back(pt);
					existing_p.push_back(average);
				}
			}

			/*� ������������ ���������� ��������� ����� ���� ��������� ��� ���� ��������� ���.
			 *�������� �������� ��� ��������� ��������*/
			/*int pair_num = 10; //�������� ��� ��������?
			int* f_idx = new int[pair_num];
			int* s_idx = new int[pair_num];

			for (int j = 0; j < pair_num; j++)
			{
				f_idx[j] = rand() % (new_p.size() + 1);
				s_idx[j] = rand() % (new_p.size() + 1);
			}

			for (int j = 0; j < pair_num; j++)
			{
				double s = norm(existing_p[f_idx[j]] - existing_p[s_idx[j]]) / norm(new_p[f_idx[j]] - new_p[s_idx[j]]);
				scale += s;
				count++;
			}

			delete f_idx, s_idx;*/

			for (int j = 0; j < new_p.size() - 1; j++)
			{
				for (int k = j + 1; k < new_p.size(); k++)
				{
					double s = norm(existing_p[j] - existing_p[k]) / norm(new_p[j] - new_p[k]);
					scale += s;
					count++;
				}
			}

			scale /= count;
			cout << format("Frame: %d Scale: %f Count: %d", i, scale, count);

			local_t *= scale; //������������ ������ � ������������� �������
			//TODO: ����������� � ������������� ����
			Mat T = Mat::eye(4, 4, CV_64F);
			local_R.copyTo(T(Range(0, 3), Range(0, 3)));
			local_t.copyTo(T(Range(0, 3), Range(3, 4)));
			curr.T = prev.T * T;

			Mat P(3, 4, CV_64F);
			Mat R = curr.T(Range(0, 3), Range(0, 3)).clone();
			Mat t = curr.T(Range(0, 3), Range(3, 4)).clone();
			((Mat)(-R.t() * t)).copyTo(P(Range(0, 3), Range(3, 4)));
			((Mat)(R.t())).copyTo(P(Range(0, 3), Range(0, 3)));
			curr.P = K * P;

			triangulatePoints(prev.P, curr.P, source, destination, points4D);
		}

		//������������ "�������" �����
		for (int j = 0; j < used.size(); j++)
		{
			if (mask.at<uchar>(j))
			{
				int k = used[j];
				int index = prev.matches[k][i + 1];

				Point3d pt;
				pt.x = points4D.at<double>(0, j) / points4D.at<double>(3, j);
				pt.y = points4D.at<double>(1, j) / points4D.at<double>(3, j);
				pt.z = points4D.at<double>(2, j) / points4D.at<double>(3, j);

				if (prev.matches3d.count(k) > 0)
				{
					curr.matches3d[index] = prev.matches3d[k]; //����� ������������ � ��� ��������� ������
					points3D[prev.matches3d[k]].pt += pt;
					points3D[curr.matches3d[index]].seen++;
				}
				else {
					sp_point new_pt; //����� ��������� �����
					new_pt.pt = pt;
					points3D.push_back(new_pt);

					prev.matches3d[k] = points3D.size() - 1;
					curr.matches3d[index] = points3D.size() - 1;
				}
			}
		}
	}

	//��������� ��������� �����
	for (sp_point p : points3D)
	{
		if (p.seen >= 3) p.pt /= p.seen - 1;
	}
}