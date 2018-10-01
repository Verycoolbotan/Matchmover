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

	//TODO: разобраться с умными указателями
	Ptr<ORB> detector = ORB::create();

	for (string path : pathList)
	{
		cout << format("Processing image %d of %d", counter, size) << endl;
		camera cam;
		//TODO: защита от дурака
		cam.image = imread(path);
		detector->detectAndCompute(cam.image, noArray(), cam.kp, cam.des);
		camList.push_back(cam);
		counter++;
	}
}

void matchFeatures()
{
	/*TODO: запомнить этот вариант инстанцирования, указатель с new
	 *используем для выделения памяти в куче*/
	BFMatcher matcher(NORM_HAMMING, true);

	/*Я видел реализацию, где изображения сравниваются по принципу "каждый с каждым".
	 *Это *немного* избыточно, для видео достаточно двух следующих кадров*/
	for (int i = 0; i < camList.size() - 1; i++)
	{
		for (int j = i + 1; j < camList.size(); j++)
		{
			cout << format("Matching features; pair %d, %d", i, j) << endl;
			vector<DMatch> matches;
			matcher.match(camList[i].des, camList[j].des, matches);

			vector<Point2d> source, destination;	//Координаты для дополнительной фильтрации
			vector<int> i_kp, j_kp;	//Индексы точек для записи в map

			for (DMatch m : matches)
			{
				source.push_back(camList[i].kp[m.queryIdx].pt);
				destination.push_back(camList[j].kp[m.trainIdx].pt);
				i_kp.push_back(m.queryIdx);	//query - что ищем
				j_kp.push_back(m.trainIdx); //train - где ищем
			}

			/*crossCheck в параметрах matcher уже отсеял большую часть ложных соответствий, но
			 *нужна дополнительная фильтрация по эпиполярным ограничениям*/
			vector<uchar> mask;
			Mat F = findFundamentalMat(source, destination, FM_RANSAC, 3, 0.99, mask);

			//Пишем соответствия в map
			for (int k = 0; k < mask.size(); k++)
			{
				//Если элемент вектора != 0, то точка с индексом k удовлетворяет ограничениям
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
	/*Этот этап отнял больше всего времени и сил, и именно здесь реализация откровенно скопирована (спасибо Nghia Ho).
	 *Сначала кадры сравниваются попарно, для каждой пары камер по существенной (ориг. essential) матрице находятся
	 *относительное вращение и смещение (Hartley, Zisserman, Multiple View Geometry in Computer Vision, второе издание,
	 *стр. 258; https://courses.graphicon.ru/files/courses/vision/2010/cv_2010_13.pdf).
	 *Накапливание преобразования - просто перемножаем матрицы предыдущей и текущей камеры. Но этого недостаточно.
	 *Cмещение нормализовано, т.е. известно только его направление.
	 *Проводим триангуляцию для первой пары (1 и 2), а затем для второй (2 и 3). Используя имеющиеся словари соответствий,
	 *находим соответственные пары в каждом наборе, ищем расстояния. Отношение этих расстояний - искомый коэффициент
	 *масштабирования.*/


	 /* focal - фокусное расстояние в мм
	  * W, H - физические размеры сенсора в мм
	  * distortion - коэффициенты искажения, по умолчанию нулевые*/

	srand(time(0));
	fov = atan((W / 2) / focal) / 3.14159265 * 180;

	//Фокусное расстояние в пикселях (прямоугольный случай)
	double fx = focal * camList[0].image.cols / W;
	double fy = focal * camList[0].image.rows / H;
	//Координаты центра, он же principal point
	double cx = camList[0].image.cols / 2;
	double cy = camList[0].image.rows / 2;

	//Получаем матрицу внутренних параметров
	Mat K = Mat::eye(3, 3, CV_64F);
	K.at<double>(0, 0) = fx;
	K.at<double>(1, 1) = fy;
	K.at<double>(0, 2) = cx;
	K.at<double>(1, 2) = cy;

	camList[0].T = Mat::eye(4, 4, CV_64F);
	camList[0].P = K * Mat::eye(3, 4, CV_64F);

	//Восстанавливаем движение между парами соседних кадров
	for (int i = 0; i < camList.size() - 1; i++)
	{
		camera& prev = camList[i];
		camera& curr = camList[i + 1];
		vector<Point2d> source, destination;
		vector<int> used;

		for (int j = 0; j < prev.kp.size(); j++)
		{
			if (prev.matches[j][i + 1]) //Если существует соответствие точке j на i+1-м кадре
			{
				int index = prev.matches[j][i + 1];
				//то запоминаем её
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
		//Нельзя просто так взять и написать индекс
		local_t.copyTo(T(Range(0, 3), Range(3, 4)));
		//Зато можно перегрузить умножение
		curr.T = prev.T * T; //Накапливаем преобразование

		Mat P(3, 4, CV_64F);
		Mat R = curr.T(Range(0, 3), Range(0, 3)).clone();
		Mat t = curr.T(Range(0, 3), Range(3, 4)).clone();
		//0 = RC + t, C = -R.t() * t, где C - центр камеры
		((Mat)(-R.t() * t)).copyTo(P(Range(0, 3), Range(3, 4)));
		((Mat)(R.t())).copyTo(P(Range(0, 3), Range(0, 3)));
		curr.P = K * P; //Новая матрица проекции

		Mat points4D;
		triangulatePoints(prev.P, curr.P, source, destination, points4D);

		/*Масштабируем всё так, чтобы соответствующие точки совпали.
		 *Используем отношение расстояний между парными точками */
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
				//Находим пары соответствующих точек, между которыми будем искать расстояние
				int k = used[j];

				if (mask.at<uchar>(j) && prev.matches[k].count(i + 1) > 0 && prev.matches3d.count(k) > 0)
				{
					//Это проще, чем использовать cv::convertPointsFromHomogeneous
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

			/*В оригинальной реализации алгоритма автор ищет отношение для всех возможных пар.
			 *Попробую заменить это случайной выборкой*/
			/*int pair_num = 10; //Добавить как параметр?
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

			local_t *= scale; //Масштабируем вектор и пересчитываем матрицы
			//TODO: разобраться с дублированием кода
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

		//Триангуляция "хороших" точек
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
					curr.matches3d[index] = prev.matches3d[k]; //Нашли соответствие с уже найденной точкой
					points3D[prev.matches3d[k]].pt += pt;
					points3D[curr.matches3d[index]].seen++;
				}
				else {
					sp_point new_pt; //Иначе добавляем новую
					new_pt.pt = pt;
					points3D.push_back(new_pt);

					prev.matches3d[k] = points3D.size() - 1;
					curr.matches3d[index] = points3D.size() - 1;
				}
			}
		}
	}

	//Усредняем положение точек
	for (sp_point p : points3D)
	{
		if (p.seen >= 3) p.pt /= p.seen - 1;
	}
}