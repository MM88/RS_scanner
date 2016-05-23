//
// Created by miky on 23/05/16.
//
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/io/vtk_io.h>
#include <pcl/surface/mls.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/console/parse.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/surface/vtk_smoothing/vtk_mesh_smoothing_laplacian.h>
#include <pcl/surface/vtk_smoothing/vtk_utils.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/surface/vtk_smoothing/vtk_mesh_smoothing_windowed_sinc.h>
#include <pcl/surface/vtk_smoothing/vtk_mesh_subdivision.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/passthrough.h>
#include <cv.h>
#include <pcl/features/boundary.h>
#include <pcl/console/time.h>
#include <pcl/registration/transforms.h>


#include "CloudProcessing.h"


void CloudProcessing::point_cloud_maker() {

    std::string data;
    std::ifstream in(txt_path);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1 (new pcl::PointCloud<pcl::PointXYZ>);

    if (in.is_open()){
        getline(in, data);
        while (!in.eof() ) {
            std::vector<std::string> x;
            boost::split(x,data, boost::is_any_of("\t "));
            pcl::PointXYZ p;
            std::stringstream ss;
            ss << x[0];
            ss >> p.x;
            ss.clear();
            ss << x[1];
            ss >> p.y;
            ss.clear();
            ss << x[2];
            ss >> p.z;
            cloud1->push_back(p);
            getline(in, data);

        }
        in.close();
    }
    cloud = cloud1;
}

void CloudProcessing::point_cloud_smoothing() {

    //MLS SMOOTHING
    MovingLeastSquares<PointXYZ, PointXYZ> mls;
    mls.setInputCloud (cloud);
    mls.setSearchRadius (0.004);
    mls.setPolynomialFit (true);
    mls.setPolynomialOrder (2);
    PointCloud<PointXYZ>::Ptr cloud_smoothed (new PointCloud<PointXYZ> ());
    mls.process (*cloud_smoothed);
    //remove nans
    for(int i=0;i<cloud_smoothed->points.size();i++){
        if (!(pcl::isFinite(cloud_smoothed->at(i)))){
            cloud_smoothed->at(i).x = 0.0;
            cloud_smoothed->at(i).y = 0.0;
            cloud_smoothed->at(i).z = 0.0;
        }
    }
    cloud = cloud_smoothed;
}

void CloudProcessing::point_cloud_filtering() {

    pcl::PassThrough<pcl::PointXYZ> pass ;
    pass.setInputCloud(cloud) ;

    pass.setFilterFieldName("z" ) ;
    pass.setFilterLimits(0.0,0.35);
    pass.filter(*cloud);

    pass.setInputCloud(cloud) ;
    pass.setFilterFieldName("x" ) ;
    pass.setFilterLimits(-0.17, 0.12);
    pass.filter(*cloud);

    pass.setInputCloud(cloud) ;
    pass.setFilterFieldName("y" ) ;
    pass.setFilterLimits(-0.09, 0.2);
    pass.filter(*cloud);

    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud (cloud);
    sor.setMeanK (20);
    sor.setStddevMulThresh (0.2);
    sor.filter (*cloud);

}


void CloudProcessing::delete_boundaries() {

    for (int i=0;i<num_border_to_remove;i++){

        pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
        pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
        ne.setInputCloud (cloud);
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
        ne.setSearchMethod (tree);
        ne.setRadiusSearch (0.002);
        ne.compute (*normals);
        pcl::PointCloud<pcl::Boundary> boundaries;
        pcl::BoundaryEstimation<pcl::PointXYZ, pcl::Normal, pcl::Boundary> est;
        est.setInputCloud (cloud);
        est.setInputNormals (normals);
        est.setRadiusSearch (0.002);
        est.setSearchMethod (pcl::search::KdTree<pcl::PointXYZ>::Ptr (new pcl::search::KdTree<pcl::PointXYZ>));
        est.compute (boundaries);

        for(int i = 0; i < cloud->points.size(); i++)
        {
            if(boundaries[i].boundary_point == 1)
            {
                cloud->at(i).z = 0;
                cloud->at(i).x = 0;
                cloud->at(i).y = 0;
            }
        }
    }
}

pcl::PolygonMesh CloudProcessing::triangulate_cloud() {

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudfiltered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud (cloud);
    sor.setMeanK (50);
    sor.setStddevMulThresh (2.5);
    sor.filter (*cloudfiltered);
    // Normal estimation*
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
    pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud (cloudfiltered);
    n.setInputCloud (cloudfiltered);
    n.setSearchMethod (tree);
    n.setKSearch (50);
    n.compute (*normals);
    // Concatenate the XYZ and normal fields*
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);
    pcl::concatenateFields (*cloudfiltered, *normals, *cloud_with_normals);
    pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointNormal>);
    tree2->setInputCloud (cloud_with_normals);
    // Initialize objects
    pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
    pcl::PolygonMesh triangles;
    // Set the maximum distance between connected points (maximum edge length)
    gp3.setSearchRadius (0.025);
    // Set typical values for the parameters
    gp3.setMu (2.5);
    gp3.setMaximumNearestNeighbors (100);
    gp3.setMaximumSurfaceAngle(M_PI/4);
    gp3.setMinimumAngle(M_PI/18); // 10 degrees
    gp3.setMaximumAngle(2*M_PI/3); // 120 degrees
    gp3.setNormalConsistency(false);
    gp3.setConsistentVertexOrdering(true);
    // Get result
    gp3.setInputCloud (cloud_with_normals);
    gp3.setSearchMethod (tree2);
    gp3.reconstruct (triangles);
    // Additional vertex information
    std::vector<int> parts = gp3.getPartIDs();
    std::vector<int> states = gp3.getPointStates();

    return triangles;

}

void CloudProcessing::processCloud() {

    point_cloud_maker();
    if (filtering) point_cloud_filtering();
    if (remove_border) delete_boundaries();
    if (smoothing) point_cloud_smoothing();
    output_mesh = triangulate_cloud();
    pcl::io::savePLYFileBinary(outPath, output_mesh);

}
