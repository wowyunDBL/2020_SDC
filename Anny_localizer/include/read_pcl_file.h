#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <dirent.h>
#include <string>
#include <iostream>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/conversions.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>

using namespace std;
void read_pcl_file(string &file_path, pcl::PointCloud<pcl::PointXYZ>::Ptr &map);
