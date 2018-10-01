#include "Matchmover.h"
#include "Export.h"

double fov;
vector<camera> camList;
vector<sp_point> points3D;

int main()
{
	vector<string> path;
	path.push_back("C:\\WORK\\Matchmover\\Core\\desk\\_DSC2192.JPG");
	path.push_back("C:\\WORK\\Matchmover\\Core\\desk\\_DSC2193.JPG");
	path.push_back("C:\\WORK\\Matchmover\\Core\\desk\\_DSC2194.JPG");
	path.push_back("C:\\WORK\\Matchmover\\Core\\desk\\_DSC2195.JPG");

	initialize(path);
	matchFeatures();
	reconstruct(27.0, 36.0, 24.0);

	toMax(camList, points3D, "Camera.ms");

	return 0;
}