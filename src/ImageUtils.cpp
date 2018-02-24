#include <iostream>
#include <sstream>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "ImageUtils.h"

using namespace cv;
using std::string;
using std::cout;
using std::cin;
using std::endl;
using std::vector;


/**
 *	Averages a video into a png
 */
void ImageUtils::averageVideo(const std::string folder, const std::string file) {
	string filename(folder);
	filename.append("/").append(file).append(".avi");
	VideoCapture video(filename);
	if (!video.isOpened()) {
		string error = "Error when reading ";
		error.append(filename);
		throw error;
	}

	Mat frame;
	Mat avgFrame;

	string outFile(folder);
	outFile.append("/").append(file).append(".png");

	int nFrames = 0;
	for (;;) {
		video >> frame;

		
		if (frame.empty()) {
			break;
		} else {
			nFrames++;
			if (avgFrame.empty()) {
				avgFrame = Mat::zeros(frame.size(), CV_32FC3);
			}
			//avgFrame += frame;
			cv::accumulate(frame, avgFrame);
		}
	}

	if (nFrames > 0) {
		avgFrame = avgFrame / nFrames;
	}

	avgFrame.convertTo(avgFrame, CV_8U);	
	imwrite(outFile, avgFrame);

}