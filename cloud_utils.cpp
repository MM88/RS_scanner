//
// Created by miky on 10/05/16.
//
#include <pcl-1.8/pcl/visualization/cloud_viewer.h>
#include <string>
#include <iostream>
#include <pcl-1.8/pcl/io/ply_io.h>
#include <pcl-1.8/pcl/io/pcd_io.h>

#include <pcl-1.8/pcl/filters/passthrough.h>
#include <pcl-1.8/pcl/filters/statistical_outlier_removal.h>
#include <pcl-1.8/pcl/filters/voxel_grid.h>
#include <pcl-1.8/pcl/surface/mls.h>
#include <boost/thread.hpp>
#include <cv.h>
#include <highgui.h>
#include <vector>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include <pcl/PCLPointCloud2.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/boundary.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>
#include <pcl/features/normal_3d.h>

namespace CloudUtils{

    using namespace pcl;
    using namespace std;

    pcl::PointCloud<pcl::PointXYZ> point_cloud_maker(std::string filePath){

        std::ostringstream txtPath;
        txtPath << filePath << ".txt";
        std::ostringstream plyPath;
        plyPath << filePath << ".ply";
        std::string data;
        std::ifstream in(txtPath.str().c_str());
        pcl::PointCloud<pcl::PointXYZ> cloud;

        if (in.is_open())
            getline(in, data);
            while (!in.eof() ) {
                std::vector<std::string> x;
                boost::split(x,data, boost::is_any_of("\t "));
                pcl::PointXYZ p;
                p.x = atof(x[0].c_str());
                p.y = atof(x[1].c_str());
                p.z = atof(x[2].c_str());
                cloud.push_back(p);
                getline(in, data);
            }

        in.close();
        cout << "Clouds saved: " << cloud.points.size() << endl;
        cout<<cloud.height<< " "<<cloud.width<<endl;
        pcl::io::savePLYFileBinary(plyPath.str(), cloud);
        return cloud;
    }

    void point_cloud_filtering(std::string filePath){


        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1 (new pcl::PointCloud<pcl::PointXYZ>);
        std::ostringstream plyPath;
        plyPath << filePath << ".ply";
        pcl::io::loadPLYFile (plyPath.str(), *cloud1);

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloudfiltered(new pcl::PointCloud<pcl::PointXYZ>);

        pcl::PassThrough<pcl::PointXYZ> pass ;
        pass.setInputCloud(cloud1) ;

        pass.setFilterFieldName("z" ) ;
        pass.setFilterLimits(0.15,0.35);
    //    pass.setFilterLimits(2000,3500);
        pass.filter(*cloudfiltered);

        pass.setInputCloud(cloudfiltered) ;
        pass.setFilterFieldName("x" ) ;
        pass.setFilterLimits(-0.2, 0.12);
    //    pass.setFilterLimits(-2000, 1200);
        pass.filter(*cloudfiltered);
    //
        pass.setInputCloud(cloudfiltered) ;
        pass.setFilterFieldName("y" ) ;
        pass.setFilterLimits(-0.09, 0.6);
    //    pass.setFilterLimits(-900, 2000);
        pass.filter(*cloudfiltered);

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloudfiltered2(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
        sor.setInputCloud (cloudfiltered);
        sor.setMeanK (20);
        sor.setStddevMulThresh (1.0);
        sor.filter (*cloudfiltered2);

        std::ostringstream filteredCloud;
        filteredCloud << filePath<<"_filtered.ply";
        pcl::io::savePLYFileBinary(filteredCloud.str(), *cloudfiltered2);

    }

    void point_cloud_average(std::string filePath, int cloud_num){

        pcl::PointCloud<pcl::PointXYZ>::Ptr avg_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr sum_cloud(new pcl::PointCloud<pcl::PointXYZ>);

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_sum(new pcl::PointCloud<pcl::PointXYZRGB>);

        std::vector< pcl::PointCloud<pcl::PointXYZ>::Ptr> cloud_vector;

        for (int i=0; i<cloud_num; i++){
            std::ostringstream plyPath;
            plyPath << filePath<< i <<"_filtered.ply";
            pcl::PointCloud<pcl::PointXYZ>::Ptr tmp_cloud (new pcl::PointCloud<pcl::PointXYZ>);
            pcl::io::loadPLYFile (plyPath.str(), *tmp_cloud);
            cloud_vector.push_back(tmp_cloud);
        }

        for (int i=0; i<cloud_num; i++){
            if (i == 0)
                pcl::copyPointCloud(*cloud_vector[i], *sum_cloud);
            else
                *sum_cloud+=*cloud_vector[i];
        }

        std::ostringstream sumCloud;
        sumCloud << filePath<<"_sum.ply";
        pcl::io::savePLYFileBinary(sumCloud.str(), *avg_cloud);


    }

    double
    computeCloudResolution(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud)
    {
        double resolution = 0.0;
        int numberOfPoints = 0;
        int nres;
        std::vector<int> indices(2);
        std::vector<float> squaredDistances(2);
        pcl::search::KdTree<pcl::PointXYZ> tree;
        tree.setInputCloud(cloud);

        for (size_t i = 0; i < cloud->size(); ++i)
        {
            if (! pcl_isfinite((*cloud)[i].x))
                continue;

            // Considering the second neighbor since the first is the point itself.
            nres = tree.nearestKSearch(i, 2, indices, squaredDistances);
            if (nres == 2)
            {
                resolution += sqrt(squaredDistances[1]);
                ++numberOfPoints;
            }
        }
        if (numberOfPoints != 0)
            resolution /= numberOfPoints;

        return resolution;
    }


    void
    compute_boundaries (std::string filePath, int proc_num)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        std::ostringstream inFile;
        inFile << filePath <<".ply";
        pcl::io::loadPLYFile(inFile.str(), *in_cloud);
        std::ostringstream outFile;
        outFile << filePath <<"_no_borders.ply";
        pcl::io::savePLYFileBinary(outFile.str(),*in_cloud);

        for (int i=0;i<proc_num;i++){

            pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::io::loadPLYFile(outFile.str(), *out_cloud);
            pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
            pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
            ne.setInputCloud (out_cloud);
            pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
            ne.setSearchMethod (tree);
            ne.setRadiusSearch (0.02);
            ne.compute (*normals);
            pcl::PointCloud<pcl::Boundary> boundaries;
            pcl::BoundaryEstimation<pcl::PointXYZ, pcl::Normal, pcl::Boundary> est;
            est.setInputCloud (out_cloud);
            est.setInputNormals (normals);
            est.setRadiusSearch (0.02);
            est.setSearchMethod (pcl::search::KdTree<pcl::PointXYZ>::Ptr (new pcl::search::KdTree<pcl::PointXYZ>));
            est.compute (boundaries);

            for(int i = 0; i < out_cloud->points.size(); i++)
            {
                if(boundaries[i].boundary_point == 1)
                {
                    out_cloud->at(i).z = 0;
                    out_cloud->at(i).x = 0;
                    out_cloud->at(i).y = 0;
                }
            }
            pcl::io::savePLYFileBinary(outFile.str(), *out_cloud);
        }
    }

}