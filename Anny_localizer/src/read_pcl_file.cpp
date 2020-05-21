#include "read_pcl_file.h"

void read_pcl_file(string &file_path, pcl::PointCloud<pcl::PointXYZ>::Ptr &map)
{
	DIR* dirp = opendir(file_path.c_str() );	
	struct dirent *dp;
	pcl::PointCloud<pcl::PointXYZ>::Ptr map_ptr (new pcl::PointCloud<pcl::PointXYZ>() );
	while((dp = readdir(dirp)) != NULL)
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr sub_map_ptr (new pcl::PointCloud<pcl::PointXYZ>() );
		std::cout<<dp->d_name<<std::endl;
		if(strstr(dp->d_name,".pcd") != NULL)
		{
			if(pcl::io::loadPCDFile<pcl::PointXYZ> (file_path + dp->d_name, *sub_map_ptr) == -1)
				PCL_ERROR ("Couldn't read file\n");
			*map_ptr += *sub_map_ptr;
		}
	}
	(void)closedir(dirp);
	cout << "map loading complete!" << endl;
	map = map_ptr;
}
