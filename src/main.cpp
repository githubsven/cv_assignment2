#include <cstdlib>
#include <string>

#include "utilities/General.h"
#include "VoxelReconstruction.h"
#include "ImageUtils.h"

using namespace nl_uu_science_gmt;

int main(
		int argc, char** argv)
{
	ImageUtils::averageVideo("data/cam1", "background");
	ImageUtils::averageVideo("data/cam2", "background");
	ImageUtils::averageVideo("data/cam3", "background");
	ImageUtils::averageVideo("data/cam4", "background");
	VoxelReconstruction::showKeys();
	VoxelReconstruction vr("data" + std::string(PATH_SEP), 4);
	vr.run(argc, argv);

	return EXIT_SUCCESS;
}
