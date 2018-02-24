#include <cstdlib>
#include <string>

#include "utilities/General.h"
#include "VoxelReconstruction.h"
#include "ImageUtils.h"

using namespace nl_uu_science_gmt;

int main(
		int argc, char** argv)
{
	FileStorage fs;
	for (int i = 1; i <= 4; i++) {
		fs.open("data/cam" + to_string(i) + "background.png", FileStorage::READ);
		if (!fs.isOpened())
			ImageUtils::averageVideo("data/cam" + to_string(i), "background");
		fs.release();
	};
	VoxelReconstruction::showKeys();
	VoxelReconstruction vr("data" + std::string(PATH_SEP), 4);
	vr.run(argc, argv);

	return EXIT_SUCCESS;
}
