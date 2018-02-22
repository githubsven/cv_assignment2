#include <iostream>
#include <sstream>
#include <time.h>
#include <stdio.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <string>
using namespace std;

#ifndef _CRT_SECURE_NO_WARNINGS
# define _CRT_SECURE_NO_WARNINGS
#endif

using namespace cv;
using namespace std;

static void help()
{
    cout <<  "This is a camera calibration sample." << endl
    <<  "Usage: calibration configurationFile"  << endl
    <<  "Near the sample file you'll find the configuration file, which has detailed help of "
    "how to edit it.  It may be any OpenCV supported file format XML/YAML." << endl;
}
class Settings
{
public:
    Settings() : goodInput(false) {}
    enum Pattern { NOT_EXISTING, CHESSBOARD, CIRCLES_GRID, ASYMMETRIC_CIRCLES_GRID };
    enum InputType {INVALID, CAMERA, VIDEO_FILE, IMAGE_LIST};
    
    void write(FileStorage& fs) const                        //Write serialization for this class
    {
        fs << "{" << "BoardSize_Width"  << boardSize.width
        << "BoardSize_Height" << boardSize.height
        << "Square_Size"         << squareSize
        << "Calibrate_Pattern" << patternToUse
        << "Calibrate_NrOfFrameToUse" << nrFrames
        << "Calibrate_FixAspectRatio" << aspectRatio
        << "Calibrate_AssumeZeroTangentialDistortion" << calibZeroTangentDist
        << "Calibrate_FixPrincipalPointAtTheCenter" << calibFixPrincipalPoint
        
        << "Write_DetectedFeaturePoints" << bwritePoints
        << "Write_extrinsicParameters"   << bwriteExtrinsics
        << "Write_outputFileName"  << outputFileName
        
        << "Show_UndistortedImage" << showUndistorsed
        
        << "Input_FlipAroundHorizontalAxis" << flipVertical
        << "Input_Delay" << delay
        << "Input" << input
        << "}";
    }
    void read(const FileNode& node)                          //Read serialization for this class
    {
        node["BoardSize_Width" ] >> boardSize.width;
        node["BoardSize_Height"] >> boardSize.height;
        node["Calibrate_Pattern"] >> patternToUse;
        node["Square_Size"]  >> squareSize;
        node["Calibrate_NrOfFrameToUse"] >> nrFrames;
        node["Calibrate_FixAspectRatio"] >> aspectRatio;
        node["Write_DetectedFeaturePoints"] >> bwritePoints;
        node["Write_extrinsicParameters"] >> bwriteExtrinsics;
        node["Write_outputFileName"] >> outputFileName;
        node["Calibrate_AssumeZeroTangentialDistortion"] >> calibZeroTangentDist;
        node["Calibrate_FixPrincipalPointAtTheCenter"] >> calibFixPrincipalPoint;
        node["Input_FlipAroundHorizontalAxis"] >> flipVertical;
        node["Show_UndistortedImage"] >> showUndistorsed;
        node["Input"] >> input;
        node["Input_Delay"] >> delay;
        interprate();
    }
    void interprate()
    {
        goodInput = true;
        if (boardSize.width <= 0 || boardSize.height <= 0)
        {
            cerr << "Invalid Board size: " << boardSize.width << " " << boardSize.height << endl;
            goodInput = false;
        }
        if (squareSize <= 10e-6)
        {
            cerr << "Invalid square size " << squareSize << endl;
            goodInput = false;
        }
        if (nrFrames <= 0)
        {
            cerr << "Invalid number of frames " << nrFrames << endl;
            goodInput = false;
        }
        
        if (input.empty())      // Check for valid input
            inputType = INVALID;
        else
        {
            if (input[0] >= '0' && input[0] <= '9')
            {
                stringstream ss(input);
                ss >> cameraID;
                inputType = CAMERA;
            }
            else
            {
                if (isListOfImages(input) && readStringList(input, imageList))
                {
                    inputType = IMAGE_LIST;
                    nrFrames = (nrFrames < (int)imageList.size()) ? nrFrames : (int)imageList.size();
                }
                else
                    inputType = VIDEO_FILE;
            }
            if (inputType == CAMERA)
                inputCapture.open(cameraID);
            if (inputType == VIDEO_FILE)
                inputCapture.open(input);
            if (inputType != IMAGE_LIST && !inputCapture.isOpened())
                inputType = INVALID;
        }
        if (inputType == INVALID)
        {
            cerr << " Inexistent input: " << input;
            goodInput = false;
        }
        
        flag = 0;
        if(calibFixPrincipalPoint) flag |= CV_CALIB_FIX_PRINCIPAL_POINT;
        if(calibZeroTangentDist)   flag |= CV_CALIB_ZERO_TANGENT_DIST;
        if(aspectRatio)            flag |= CV_CALIB_FIX_ASPECT_RATIO;
        
        
        calibrationPattern = NOT_EXISTING;
        if (!patternToUse.compare("CHESSBOARD")) calibrationPattern = CHESSBOARD;
        if (!patternToUse.compare("CIRCLES_GRID")) calibrationPattern = CIRCLES_GRID;
        if (!patternToUse.compare("ASYMMETRIC_CIRCLES_GRID")) calibrationPattern = ASYMMETRIC_CIRCLES_GRID;
        if (calibrationPattern == NOT_EXISTING)
        {
            cerr << " Inexistent camera calibration mode: " << patternToUse << endl;
            goodInput = false;
        }
        atImageList = 0;
        
    }
    Mat nextImage()
    {
        Mat result;
        if( inputCapture.isOpened() )
        {
            Mat view0;
            inputCapture >> view0;
            view0.copyTo(result);
        }
        else if( atImageList < (int)imageList.size() )
            result = imread(imageList[atImageList++], CV_LOAD_IMAGE_COLOR);
        
        return result;
    }
    
    static bool readStringList( const string& filename, vector<string>& l )
    {
        l.clear();
        FileStorage fs(filename, FileStorage::READ);
        if( !fs.isOpened() )
            return false;
        FileNode n = fs.getFirstTopLevelNode();
        if( n.type() != FileNode::SEQ )
            return false;
        FileNodeIterator it = n.begin(), it_end = n.end();
        for( ; it != it_end; ++it )
            l.push_back((string)*it);
        return true;
    }
    
    static bool isListOfImages( const string& filename)
    {
        string s(filename);
        // Look for file extension
        if( s.find(".xml") == string::npos && s.find(".yaml") == string::npos && s.find(".yml") == string::npos )
            return false;
        else
            return true;
    }
public:
    Size boardSize;            // The size of the board -> Number of items by width and height
    Pattern calibrationPattern;// One of the Chessboard, circles, or asymmetric circle pattern
    float squareSize;          // The size of a square in your defined unit (point, millimeter,etc).
    int nrFrames;              // The number of frames to use from the input for calibration
    float aspectRatio;         // The aspect ratio
    int delay;                 // In case of a video input
    bool bwritePoints;         //  Write detected feature points
    bool bwriteExtrinsics;     // Write extrinsic parameters
    bool calibZeroTangentDist; // Assume zero tangential distortion
    bool calibFixPrincipalPoint;// Fix the principal point at the center
    bool flipVertical;          // Flip the captured images around the horizontal axis
    string outputFileName;      // The name of the file where to write
    bool showUndistorsed;       // Show undistorted images after calibration
    string input;               // The input ->
    
    
    
    int cameraID;
    vector<string> imageList;
    int atImageList;
    VideoCapture inputCapture;
    InputType inputType;
    bool goodInput;
    int flag;
    
private:
    string patternToUse;
    
    
};

static void read(const FileNode& node, Settings& x, const Settings& default_value = Settings())
{
    if(node.empty())
        x = default_value;
    else
        x.read(node);
}

enum { DETECTION = 0, CAPTURING = 1, CALIBRATED = 2 };

bool runCalibrationAndSave(Settings& s, Size imageSize, Mat&  cameraMatrix, Mat& distCoeffs,
                           vector<vector<Point2f> > imagePoints );

void drawCube(Mat &view, Mat rvec, Mat tvec, Mat cameraMatrix, Mat distCoeffs);
void drawAxis(Mat &view, Mat rvec, Mat tvec, Mat cameraMatrix, Mat distCoeffs);

int main(int argc, char* argv[])
{
    help();
    Settings s;
    const string inputSettingsFile = argc > 1 ? argv[1] : "default.xml";
    FileStorage fs(inputSettingsFile, FileStorage::READ); // Read the settings
    if (!fs.isOpened())
    {
        cout << "Could not open the configuration file: \"" << inputSettingsFile << "\"" << endl;
        return -1;
    }
    fs["Settings"] >> s;
    fs.release();                                         // close Settings file
    
    if (!s.goodInput)
    {
        cout << "Invalid input detected. Application stopping. " << endl;
        return -1;
    }
    
    vector<vector<Point2f> > imagePoints;
    Mat cameraMatrix, distCoeffs;
    Size imageSize;
    int mode = s.inputType == Settings::IMAGE_LIST ? CAPTURING : DETECTION;
    clock_t prevTimestamp = 0;
    const Scalar RED(0,0,255), GREEN(0,255,0);
    const char ESC_KEY = 27;
    
    for(int i = 0;;++i)
    {
        Mat view;
        bool blinkOutput = false;
        
        view = s.nextImage();
        
        //-----  If no more image, or got enough, then stop calibration and show result -------------
        if( mode == CAPTURING && imagePoints.size() >= (unsigned)s.nrFrames )
        {
            if( runCalibrationAndSave(s, imageSize,  cameraMatrix, distCoeffs, imagePoints))
                mode = CALIBRATED;
            else
                mode = DETECTION;
        }
        if(view.empty())          // If no more images then run calibration, save and stop loop.
        {
            if( imagePoints.size() > 0 )
                runCalibrationAndSave(s, imageSize,  cameraMatrix, distCoeffs, imagePoints);
            break;
        }
        
        
        imageSize = view.size();  // Format input image.
        if( s.flipVertical )    flip( view, view, 0 );
        
        vector<Point2f> pointBuf;
        
        bool found;
        switch( s.calibrationPattern ) // Find feature points on the input format
        {
            case Settings::CHESSBOARD:
                found = findChessboardCorners( view, s.boardSize, pointBuf,
                                              CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FAST_CHECK | CV_CALIB_CB_NORMALIZE_IMAGE);
                break;
            case Settings::CIRCLES_GRID:
                found = findCirclesGrid( view, s.boardSize, pointBuf );
                break;
            case Settings::ASYMMETRIC_CIRCLES_GRID:
                found = findCirclesGrid( view, s.boardSize, pointBuf, CALIB_CB_ASYMMETRIC_GRID );
                break;
            default:
                found = false;
                break;
        }
        
        if ( found)                // If done with success,
        {
            // improve the found corners' coordinate accuracy for chessboard
            if( s.calibrationPattern == Settings::CHESSBOARD)
            {
                Mat viewGray;
                cvtColor(view, viewGray, COLOR_BGR2GRAY);
                cornerSubPix( viewGray, pointBuf, Size(11,11),
                             Size(-1,-1), TermCriteria( CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 30, 0.1 ));
            }
            
            if( mode == CAPTURING &&  // For camera only take new samples after delay time
               (!s.inputCapture.isOpened() || clock() - prevTimestamp > s.delay*1e-3*CLOCKS_PER_SEC) )
            {
                imagePoints.push_back(pointBuf);
                prevTimestamp = clock();
                blinkOutput = s.inputCapture.isOpened();
            }
            
            // Draw the corners.
            // drawChessboardCorners( view, s.boardSize, Mat(pointBuf), found );
        }
        
        //----------------------------- Output Text ------------------------------------------------
        string msg = (mode == CAPTURING) ? "100/100" :
        mode == CALIBRATED ? "Calibrated" : "Press 'g' to start";
        int baseLine = 0;
        Size textSize = getTextSize(msg, 1, 1, 1, &baseLine);
        Point textOrigin(view.cols - 2*textSize.width - 10, view.rows - 2*baseLine - 10);
        
        if( mode == CAPTURING )
        {
            if(s.showUndistorsed)
                msg = format( "%d/%d Undist", (int)imagePoints.size(), s.nrFrames );
            else
                msg = format( "%d/%d", (int)imagePoints.size(), s.nrFrames );
        }
        
        putText( view, msg, textOrigin, 1, 1, mode == CALIBRATED ?  GREEN : RED);
        
        //------------------------------ Draw   point   positions -----------------------------------
        /* for (int i = 0; i < pointBuf.size(); i++) {
            string location = to_string((int)pointBuf[i].x) + ", " + to_string((int)pointBuf[i].y);
            Point position = Point(pointBuf[i].x + 7, pointBuf[i].y);
            putText( view, location , position, 1, .8f, GREEN);
        } */
        
        //------------------------------- Draw    basic    square -----------------------------------
        /*if (pointBuf.size() > 0) {
            Scalar color = Scalar(255, 0, 0);
            line(view, pointBuf[7], pointBuf[8], color);
            line(view, pointBuf[8], pointBuf[17], color);
            line(view, pointBuf[16], pointBuf[17], color);
            line(view, pointBuf[7], pointBuf[16], color);
        }*/
        
		// 
        /**
		 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
		 *                                     ASSIGNMENT CODE
		 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
		 */
        if(mode == CALIBRATED && found) { // SolvePnP will crash if the pattern was not found
            vector<Point3f> objectPoints;
            for (int i = 0; i < s.boardSize.height; i++)
                for (int j = 0; j < s.boardSize.width; j++)
                    objectPoints.push_back(Point3f(i * s.squareSize, j * s.squareSize, 0)); // The 3D points that correspond to the image points
            
            Mat rvec, tvec; //rotation vector, translation vector
            solvePnP(objectPoints, pointBuf, cameraMatrix, distCoeffs, rvec, tvec); //Solve for which rvec and tvec the object points best match the image points (a.k.a. find the extrinsic parameters).
            
			
            drawAxis(view, rvec, tvec, cameraMatrix, distCoeffs); // Draws axis using cv::projectPoints
			drawCube(view, rvec, tvec, cameraMatrix, distCoeffs); // Draws cube manually
        }
        
        if( blinkOutput )
            bitwise_not(view, view);
        
        //------------------------- Video capture  output  undistorted ------------------------------
        if( mode == CALIBRATED && s.showUndistorsed )
        {
            Mat temp = view.clone();
            undistort(temp, view, cameraMatrix, distCoeffs);
        }
        
        //------------------------------ Show image and check for input commands -------------------
        imshow("Image View", view);
        char key = (char)waitKey(s.inputCapture.isOpened() ? 50 : s.delay);
        
        if( key  == ESC_KEY )
            break;
        
        if( key == 'u' && mode == CALIBRATED )
            s.showUndistorsed = !s.showUndistorsed;
        
        if( s.inputCapture.isOpened() && key == 'g' )
        {
            mode = CAPTURING;
            imagePoints.clear();
        }
    }
    
    // -----------------------Show the undistorted image for the image list ------------------------
    if( s.inputType == Settings::IMAGE_LIST && s.showUndistorsed )
    {
        Mat view, rview, map1, map2;
        initUndistortRectifyMap(cameraMatrix, distCoeffs, Mat(),
                                getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, imageSize, 1, imageSize, 0),
                                imageSize, CV_16SC2, map1, map2);
        
        for(int i = 0; i < (int)s.imageList.size(); i++ )
        {
            view = imread(s.imageList[i], 1);
            if(view.empty())
                continue;
            remap(view, rview, map1, map2, INTER_LINEAR);
            imshow("Image View", rview);
            char c = (char)waitKey();
            if( c  == ESC_KEY || c == 'q' || c == 'Q' )
                break;
        }
    }
    
    
    return 0;
}


/**
 *	Projects a list of points from the 3D world coordinates to the image coordinates.
 *	The calculations are done step by step by ourselves
 */
void manualProjectPoints(vector<Point3f> objectPoints, Mat rvec, Mat tvec, Mat cameraMatrix, Mat distCoeffs, vector<Point2f> & imagePoints) {
	Mat_<double> rMat;
	Rodrigues(rvec, rMat);

	double extrinsicParametersData[3][4] = {
		{ rMat.at<double>(0,0), rMat.at<double>(0,1), rMat.at<double>(0,2), tvec.at<double>(0,0)},
		{ rMat.at<double>(1,0), rMat.at<double>(1,1), rMat.at<double>(1,2), tvec.at<double>(1,0)},
		{ rMat.at<double>(2,0), rMat.at<double>(2,1), rMat.at<double>(2,2), tvec.at<double>(2,0)}
	};
	Mat extrinsicParameters = Mat(3, 4, CV_64F, &extrinsicParametersData);
	int size = objectPoints.size();
	for (int i = 0; i < objectPoints.size(); i++) {
		Point3f point = objectPoints.at(i);
		double pointData[4][1] = {
			{point.x},
			{point.y},
			{point.z},
			{1}		
		};
		Mat_<double> pointMat = Mat(4, 1, CV_64F, &pointData);
		Mat_<double> intermediatePointMat = extrinsicParameters * pointMat;
		double x = intermediatePointMat.at<double>(0, 0);
		double y = intermediatePointMat.at<double>(1, 0);
		double z = intermediatePointMat.at<double>(2, 0);

		// Transform from Homogeneous to Inhomogeneous coordinates
		double x1 = x / z;
		double y1 = y / z;

		// Get distorsion coeficients
		double r = pow(x1, 2) + pow(y1, 2);
		
		int nCoeffs = distCoeffs.size().height;

		// Always present?
		double k1 = 0;
		double k2 = 0;
		double p1 = 0;
		double p2 = 0;

		// Optional
		double k3 = 0;

		// Optional (the three as a batch)
		double k4 = 0;
		double k5 = 0;
		double k6 = 0;

		switch (nCoeffs) {
		case 8:
			k4 = distCoeffs.at<double>(5, 0);
			k5 = distCoeffs.at<double>(6, 0);
			k6 = distCoeffs.at<double>(7, 0);
		case 5:
			k3 = distCoeffs.at<double>(4, 0);
		case 4:
			k1 = distCoeffs.at<double>(0, 0);
			k2 = distCoeffs.at<double>(1, 0);
			p1 = distCoeffs.at<double>(2, 0);
			p2 = distCoeffs.at<double>(3, 0);
		default:
			break; // Do nothing
		}

		// Apply distorsion coeficients
		// If there were none (size == 0 and all coefficients initialized as 0), these formulas will result in x2 = x1 and y2 = y1
		double x2 = x1 * ((1 + k1 * pow(r, 2) + k2 * pow(r, 4) + k3 * pow(r, 6)) / (1 + k4 * pow(r, 2) + k5 * pow(r, 4) + k6 * pow(r, 6)))
			+ 2 * p1 * x1 * y1
			+ p2 * (pow(r, 2) + 2 * pow(x1, 2));
		double y2 = y1 * ((1 + k1 * pow(r, 2) + k2 * pow(r, 4) + k3 * pow(r, 6)) / (1 + k4 * pow(r, 2) + k5 * pow(r, 4) + k6 * pow(r, 6)))
			+ p1 * (pow(r, 2) + 2 * pow(y1, 2))
			+ 2 * p2 * x1 * y1;
		
		double newIntermediatePointData[3][1] = {
			{x2},
			{y2},
			{1}
		};
		Mat_<double> newIntermediatePoint = Mat(3, 1, CV_64F, &newIntermediatePointData);
		Mat_<double> projectedPointMat = cameraMatrix * newIntermediatePoint;
		double u = projectedPointMat.at<double>(0, 0);
		double v = projectedPointMat.at<double>(1, 0);
		double w = projectedPointMat.at<double>(2, 0);

		Point2f projectedPoint = Point2f(u, v);

		imagePoints.push_back(projectedPoint);
	}
	
}

/**
 *	Draws a cube at the origin of the world coordinates
 */
void drawCube(Mat &view, Mat rvec, Mat tvec, Mat cameraMatrix, Mat distCoeffs) {
    vector<Point3f> cube(8); //Defines the corners of the cube
	cube[0] = Point3f(0, 0, 0);
	cube[1] = Point3f(0, 150, 0);
	cube[2] = Point3f(150, 150, 0);
	cube[3] = Point3f(150, 0, 0);
	cube[4] = Point3f(0, 0, 150);
	cube[5] = Point3f(0, 150, 150);
	cube[6] = Point3f(150, 150, 150);
	cube[7] = Point3f(150, 0, 150);

	vector<Point2f> imgPts; // Projected 3D cube corners
	manualProjectPoints(cube, rvec, tvec, cameraMatrix, distCoeffs, imgPts);
    
	Scalar color = Scalar(103, 65, 203); //A nice shade of pink
	int thickness = 2;

    //Draw bottom square
    for (int i = 1; i < 4; i ++)
        line(view, imgPts[i - 1], imgPts[i], color, thickness);
    line(view, imgPts[0], imgPts[3], color, thickness);
    
    //Draw lines from bottom square to top square
    for (int i = 4; i < 8; i ++)
        line(view, imgPts[i - 4], imgPts[i], color, thickness);
    
    //Draw top square
    for (int i = 5; i < 8; i ++)
        line(view, imgPts[i - 1], imgPts[i], color, thickness);
    line(view, imgPts[4], imgPts[7], color, thickness);
}

/**
 *	Draws the axis at the origin of the world coordinates
 */
void drawAxis(Mat &view, Mat rvec, Mat tvec, Mat cameraMatrix, Mat distCoeffs) {
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
}

static double computeReprojectionErrors( const vector<vector<Point3f> >& objectPoints,
                                        const vector<vector<Point2f> >& imagePoints,
                                        const vector<Mat>& rvecs, const vector<Mat>& tvecs,
                                        const Mat& cameraMatrix , const Mat& distCoeffs,
                                        vector<float>& perViewErrors)
{
    vector<Point2f> imagePoints2;
    int i, totalPoints = 0;
    double totalErr = 0, err;
    perViewErrors.resize(objectPoints.size());
    
    for( i = 0; i < (int)objectPoints.size(); ++i )
    {
        projectPoints( Mat(objectPoints[i]), rvecs[i], tvecs[i], cameraMatrix,
                      distCoeffs, imagePoints2);
        err = norm(Mat(imagePoints[i]), Mat(imagePoints2), CV_L2);
        
        int n = (int)objectPoints[i].size();
        perViewErrors[i] = (float) std::sqrt(err*err/n);
        totalErr        += err*err;
        totalPoints     += n;
    }
    
    return std::sqrt(totalErr/totalPoints);
}

static void calcBoardCornerPositions(Size boardSize, float squareSize, vector<Point3f>& corners,
                                     Settings::Pattern patternType /*= Settings::CHESSBOARD*/)
{
    corners.clear();
    
    switch(patternType)
    {
        case Settings::CHESSBOARD:
        case Settings::CIRCLES_GRID:
            for( int i = 0; i < boardSize.height; ++i )
                for( int j = 0; j < boardSize.width; ++j )
                    corners.push_back(Point3f(float( j*squareSize ), float( i*squareSize ), 0));
            break;
            
        case Settings::ASYMMETRIC_CIRCLES_GRID:
            for( int i = 0; i < boardSize.height; i++ )
                for( int j = 0; j < boardSize.width; j++ )
                    corners.push_back(Point3f(float((2*j + i % 2)*squareSize), float(i*squareSize), 0));
            break;
        default:
            break;
    }
}

static bool runCalibration( Settings& s, Size& imageSize, Mat& cameraMatrix, Mat& distCoeffs,
                           vector<vector<Point2f> > imagePoints, vector<Mat>& rvecs, vector<Mat>& tvecs,
                           vector<float>& reprojErrs,  double& totalAvgErr)
{
    
    cameraMatrix = Mat::eye(3, 3, CV_64F);
    if( s.flag & CV_CALIB_FIX_ASPECT_RATIO )
        cameraMatrix.at<double>(0,0) = 1.0;
    
    distCoeffs = Mat::zeros(8, 1, CV_64F);
    
    vector<vector<Point3f> > objectPoints(1);
    calcBoardCornerPositions(s.boardSize, s.squareSize, objectPoints[0], s.calibrationPattern);
    
    objectPoints.resize(imagePoints.size(),objectPoints[0]);
    cout << "T";
    //Find intrinsic and extrinsic camera parameters
    double rms = calibrateCamera(objectPoints, imagePoints, imageSize, cameraMatrix,
                                 distCoeffs, rvecs, tvecs, s.flag|CV_CALIB_FIX_K4|CV_CALIB_FIX_K5);
    
    cout << "Re-projection error reported by calibrateCamera: "<< rms << endl;
    
    bool ok = checkRange(cameraMatrix) && checkRange(distCoeffs);
    
    totalAvgErr = computeReprojectionErrors(objectPoints, imagePoints,
                                            rvecs, tvecs, cameraMatrix, distCoeffs, reprojErrs);
    
    return ok;
}

// Print camera parameters to the output file
static void saveCameraParams( Settings& s, Size& imageSize, Mat& cameraMatrix, Mat& distCoeffs,
                             const vector<Mat>& rvecs, const vector<Mat>& tvecs,
                             const vector<float>& reprojErrs, const vector<vector<Point2f> >& imagePoints,
                             double totalAvgErr )
{
    FileStorage fs( s.outputFileName, FileStorage::WRITE );
    
    time_t tm;
    time( &tm );
	struct tm *t2 =localtime(&tm);
    char buf[1024];
    strftime( buf, sizeof(buf)-1, "%c", t2 );
    
    fs << "calibration_Time" << buf;
    
    if( !rvecs.empty() || !reprojErrs.empty() )
        fs << "nrOfFrames" << (int)std::max(rvecs.size(), reprojErrs.size());
    fs << "image_Width" << imageSize.width;
    fs << "image_Height" << imageSize.height;
    fs << "board_Width" << s.boardSize.width;
    fs << "board_Height" << s.boardSize.height;
    fs << "square_Size" << s.squareSize;
    
    if( s.flag & CV_CALIB_FIX_ASPECT_RATIO )
        fs << "FixAspectRatio" << s.aspectRatio;
    
    if( s.flag )
    {
        sprintf( buf, "flags: %s%s%s%s",
                s.flag & CV_CALIB_USE_INTRINSIC_GUESS ? " +use_intrinsic_guess" : "",
                s.flag & CV_CALIB_FIX_ASPECT_RATIO ? " +fix_aspectRatio" : "",
                s.flag & CV_CALIB_FIX_PRINCIPAL_POINT ? " +fix_principal_point" : "",
                s.flag & CV_CALIB_ZERO_TANGENT_DIST ? " +zero_tangent_dist" : "" );
        cvWriteComment( *fs, buf, 0 );
        
    }
    
    fs << "flagValue" << s.flag;
    
    fs << "Camera_Matrix" << cameraMatrix;
    fs << "Distortion_Coefficients" << distCoeffs;
    
    fs << "Avg_Reprojection_Error" << totalAvgErr;
    if( !reprojErrs.empty() )
        fs << "Per_View_Reprojection_Errors" << Mat(reprojErrs);
    
    if( !rvecs.empty() && !tvecs.empty() )
    {
        CV_Assert(rvecs[0].type() == tvecs[0].type());
        Mat bigmat((int)rvecs.size(), 6, rvecs[0].type());
        for( int i = 0; i < (int)rvecs.size(); i++ )
        {
            Mat r = bigmat(Range(i, i+1), Range(0,3));
            Mat t = bigmat(Range(i, i+1), Range(3,6));
            
            CV_Assert(rvecs[i].rows == 3 && rvecs[i].cols == 1);
            CV_Assert(tvecs[i].rows == 3 && tvecs[i].cols == 1);
            //*.t() is MatExpr (not Mat) so we can use assignment operator
            r = rvecs[i].t();
            t = tvecs[i].t();
        }
        cvWriteComment( *fs, "a set of 6-tuples (rotation vector + translation vector) for each view", 0 );
        fs << "Extrinsic_Parameters" << bigmat;
    }
    
    if( !imagePoints.empty() )
    {
        Mat imagePtMat((int)imagePoints.size(), (int)imagePoints[0].size(), CV_32FC2);
        for( int i = 0; i < (int)imagePoints.size(); i++ )
        {
            Mat r = imagePtMat.row(i).reshape(2, imagePtMat.cols);
            Mat imgpti(imagePoints[i]);
            imgpti.copyTo(r);
        }
        fs << "Image_points" << imagePtMat;
    }
}

//Calculate the camera parameters and save them to a file
bool runCalibrationAndSave(Settings& s, Size imageSize, Mat&  cameraMatrix, Mat& distCoeffs,vector<vector<Point2f> > imagePoints )
{
    vector<Mat> rvecs, tvecs;
    vector<float> reprojErrs;
    double totalAvgErr = 0;
    
    bool ok = runCalibration(s,imageSize, cameraMatrix, distCoeffs, imagePoints, rvecs, tvecs,
                             reprojErrs, totalAvgErr);
    cout << (ok ? "Calibration succeeded" : "Calibration failed")
    << ". avg re projection error = "  << totalAvgErr ;
    
    if( ok )
        saveCameraParams( s, imageSize, cameraMatrix, distCoeffs, rvecs ,tvecs, reprojErrs,
                         imagePoints, totalAvgErr);
    return ok;
}

