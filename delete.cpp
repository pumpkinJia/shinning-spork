#include <iostream>
#include <algorithm>
#include <vector>
#include <pcl/io/pcd_io.h>  
#include <pcl/point_types.h>  
#include <pcl/kdtree/kdtree_flann.h>  //kdtree��������
#include <pcl/filters/extract_indices.h>
#include <boost/thread/thread.hpp>
#include <pcl/visualization/pcl_visualizer.h>

using namespace std;

int main()
{
	//----------------------��ȡ����---------------------
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	if (pcl::io::loadPCDFile<pcl::PointXYZ>("Transformed.pcd", *cloud) == -1)
	{
		PCL_ERROR("Cloudn't read file!");
		return -1;
	}
	//---------------------KD���뾶����-------------------
	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
	kdtree.setInputCloud(cloud);
	vector<int> pointIdxRadiusSearch;          //����ÿ�����ڵ������
	vector<float> pointRadiusSquaredDistance;  //����ÿ�����ڵ�����ҵ�֮���ŷʽ����ƽ��
	vector<int> total_index;
	float radius = 0.0005;//������֮��ľ���Ϊ0.000001����Ϊ���غϵ�
	/*��ĳһ����0.000001�����ڲ�ֹ�䱾��һ���㣬����Ϊ�����ظ��㡣
	���ظ����������¼���������ں����Դ��ظ���Ϊ��ѯ������ʱ����ʱ��һ��Ҳ�ᱻ����Ϊ�ظ��㣬
	��pointIdxRadiusSearch�ж����������еģ��ʴ�pointIdxRadiusSearch�еĵڶ������������ʼ��¼��
	�������Ա�֤����ɾ���ظ��ĵ㣬������һ����*/
	for (size_t i = 0; i < cloud->size(); ++i)//��cloud�е�ÿ�����������ڵĵ���бȽ�
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
	//-----------------------ɾ���ظ�����-----------------------
	sort(total_index.begin(), total_index.end());//��������������
	total_index.erase(unique(total_index.begin(), total_index.end()), total_index.end());//�������е��ظ�����ȥ��

	//-------------------��������ɾ���ظ��ĵ�-------------------
	pcl::PointIndices::Ptr outliners(new pcl::PointIndices());
	outliners->indices.resize(total_index.size());
	for (size_t i = 0; i < total_index.size(); i++)
	{
		outliners->indices[i] = total_index[i];
	}
	cout << "�ظ�����ɾ����ϣ�����" << endl;
	//-------------------��ȡɾ���ظ���֮��ĵ���--------------
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::ExtractIndices<pcl::PointXYZ> extract;
	extract.setInputCloud(cloud);
	extract.setIndices(outliners);
	extract.setNegative(true);//����Ϊtrue���ʾ��������֮��ĵ�
	extract.filter(*cloud_filtered);
	cout << "ԭʼ�����е�ĸ���Ϊ��" << cloud->points.size() << endl;
	cout << "ɾ�����ظ���ĸ���Ϊ:" << total_index.size() << endl;
	cout << "ȥ��֮���ĸ���Ϊ:" << cloud_filtered->points.size() << endl;
	//-------------------------���ӻ�-------------------------
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
