#include <cstdlib>
#include <string>

#include "utilities/General.h"
#include "VoxelReconstruction.h"
#include "calibrate.h"
#include "background.h"

using namespace nl_uu_science_gmt;

int main(
		int argc, char** argv)
{
//	calibrate::runCalibration();
	Background::averageBackground("data/cam1", "background.avi");
	Background::averageBackground("data/cam2", "background.avi");
	Background::averageBackground("data/cam3", "background.avi");
	Background::averageBackground("data/cam4", "background.avi");
	VoxelReconstruction::showKeys();
	VoxelReconstruction vr("data" + std::string(PATH_SEP), 4);
	vr.run(argc, argv);

	return EXIT_SUCCESS;
}
