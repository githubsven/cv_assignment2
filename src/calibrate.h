#pragma once
#include <iostream>
#include <sstream>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace cv;
using namespace std;

static class calibrate {
public:
	static void runCalibration(Size& imageSize, Mat& cameraMatrix, Mat& distCoeffs, vector<vector<Point3f>> objectPoints,
		vector<vector<Point2f> > imagePoints, vector<Mat>& rvecs, vector<Mat>& tvecs,
		vector<float>& reprojErrs, double& totalAvgErr);
};