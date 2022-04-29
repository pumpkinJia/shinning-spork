#include <iostream>
#include <algorithm>
#include <vector>
#include <pcl/io/pcd_io.h>  
#include <pcl/point_types.h>  
#include <pcl/kdtree/kdtree_flann.h>  //kdtree近邻搜索
#include <pcl/filters/extract_indices.h>
#include <boost/thread/thread.hpp>
#include <pcl/visualization/pcl_visualizer.h>

using namespace std;

int main()
{
	//----------------------读取点云---------------------
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	if (pcl::io::loadPCDFile<pcl::PointXYZ>("Transformed.pcd", *cloud) == -1)
	{
		PCL_ERROR("Cloudn't read file!");
		return -1;
	}
	//---------------------KD树半径搜索-------------------
	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
	kdtree.setInputCloud(cloud);
	vector<int> pointIdxRadiusSearch;          //保存每个近邻点的索引
	vector<float> pointRadiusSquaredDistance;  //保存每个近邻点与查找点之间的欧式距离平方
	vector<int> total_index;
	float radius = 0.0005;//若两点之间的距离为0.000001则认为是重合点
	/*若某一点在0.000001领域内不止其本身一个点，则认为其有重复点。
	将重复点的索引记录下来，由于后续以此重复点为查询点搜索时，此时这一点也会被定义为重复点，
	但pointIdxRadiusSearch中都是升序排列的，故从pointIdxRadiusSearch中的第二个点的索引开始记录，
	这样可以保证仅仅删除重复的点，保留第一个点*/
	for (size_t i = 0; i < cloud->size(); ++i)//对cloud中的每个点与邻域内的点进行比较
	{
		pcl::PointXYZ searchPoint = cloud->points[i];

		if (kdtree.radiusSearch(searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0)
		{
			if (pointIdxRadiusSearch.size() != 1)
			{
				for (size_t j = 1; j < pointIdxRadiusSearch.size(); j++)
				{
					total_index.push_back(pointIdxRadiusSearch[j]);
				}
			}
		}
	}
	//-----------------------删除重复索引-----------------------
	sort(total_index.begin(), total_index.end());//将索引进行排序
	total_index.erase(unique(total_index.begin(), total_index.end()), total_index.end());//将索引中的重复索引去除

	//-------------------根据索引删除重复的点-------------------
	pcl::PointIndices::Ptr outliners(new pcl::PointIndices());
	outliners->indices.resize(total_index.size());
	for (size_t i = 0; i < total_index.size(); i++)
	{
		outliners->indices[i] = total_index[i];
	}
	cout << "重复点云删除完毕！！！" << endl;
	//-------------------提取删除重复点之后的点云--------------
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::ExtractIndices<pcl::PointXYZ> extract;
	extract.setInputCloud(cloud);
	extract.setIndices(outliners);
	extract.setNegative(true);//设置为true则表示保存索引之外的点
	extract.filter(*cloud_filtered);
	cout << "原始点云中点的个数为：" << cloud->points.size() << endl;
	cout << "删除的重复点的个数为:" << total_index.size() << endl;
	cout << "去重之后点的个数为:" << cloud_filtered->points.size() << endl;
	//-------------------------可视化-------------------------
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(cloud_filtered, 0, 255, 0); // green

	viewer->addPointCloud<pcl::PointXYZ>(cloud_filtered, single_color, "sample cloud");

	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}

	return 0;
}
