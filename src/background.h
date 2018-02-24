#ifndef BACKGROUND_UTILS_
#define BACKGROUND_UTILS_

#include <iostream>
#include <sstream>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace cv;
using namespace std;

class Background {
public: 
	static void averageBackground(const std::string folder, const std::string file);
};

#endif /* BACKGROUND_UTILS_ */