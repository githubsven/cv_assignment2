/*
 * General.cpp
 *
 *  Created on: Nov 13, 2013
 *      Author: coert
 */

#include "General.h"

#include <fstream>

using namespace std;

namespace nl_uu_science_gmt
{

const string General::CBConfigFile         = "checkerboard.xml";
const string General::CalibrationVideo     = "calibration.avi";
const string General::CheckerboadVideo     = "checkerboard.avi";
const string General::BackgroundImageFile  = "background.png";
const string General::VideoFile            = "video.avi";
const string General::IntrinsicsFile       = "intrinsics.xml";
const string General::CheckerboadCorners   = "boardcorners.xml";
const string General::ConfigFile           = "config.xml";
const string General::IntrinsicsVideo	   = "intrinsics.avi";

/**
 * Linux/Windows friendly way to check if a file exists
 */
bool General::fexists(const std::string &filename)
{
	ifstream ifile(filename.c_str());
	return ifile.is_open();
}

/**
*	Draws the axis at the origin of the world coordinates
*/
/*void General::drawAxis(Mat &view, Mat rvec, Mat tvec, Mat cameraMatrix, Mat distCoeffs) {
	vector<Point3f> axis(4);
	axis[0] = Point3f(0, 0, 0);
	axis[1] = Point3f(250, 0, 0);
	axis[2] = Point3f(0, 250, 0);
	axis[3] = Point3f(0, 0, 250);

	vector<Point2f> imgPts; // Projected 3D cube corners
	projectPoints(axis, rvec, tvec, cameraMatrix, distCoeffs, imgPts);

	line(view, imgPts[0], imgPts[1], Scalar(0, 0, 255), 3); // draw x-axis
	line(view, imgPts[0], imgPts[2], Scalar(0, 255, 0), 3); // draw y-axis
	line(view, imgPts[0], imgPts[3], Scalar(255, 0, 0), 3); // dray z-axis
}*/

} /* namespace nl_uu_science_gmt */
