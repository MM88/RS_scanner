//
// Created by miky on 10/05/16.
//



#ifndef CLOUD_UTILS_CLOUD_UTILS_H
#define CLOUD_UTILS_CLOUD_UTILS_H

namespace CloudUtils{

    pcl::PointCloud<pcl::PointXYZ> point_cloud_maker(std::string filePath);
    void point_cloud_filtering(std::string filePath);
    void point_cloud_average(std::string filePath, int cloud_num);
    double computeCloudResolution(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud);
    void delete_boundaries(std::string filePath, int proc_num);
    void triangulate_cloud(std::string filePath);
    void point_cloud_smoothing(std::string filePath);

}


#endif //CLOUD_UTILS_CLOUD_UTILS_H
