//
// Created by miky on 10/05/16.
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
#include <pcl/surface/vtk_smoothing/vtk_utils.h>


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
                cloud.push_back(p);
                getline(in, data);

            }
            in.close();
        }

        cout << "Clouds saved: " << cloud.points.size() << endl;
        cout<<cloud.height<< " "<<cloud.width<<endl;
        pcl::io::savePLYFileBinary(plyPath.str(), cloud);
        return cloud;
    }

    void
    point_cloud_filtering(std::string filePath){

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1 (new pcl::PointCloud<pcl::PointXYZ>);
        std::ostringstream plyPath;
        plyPath << filePath << ".ply";
        pcl::io::loadPLYFile (plyPath.str(), *cloud1);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloudfiltered(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PassThrough<pcl::PointXYZ> pass ;
        pass.setInputCloud(cloud1) ;

        pass.setFilterFieldName("z" ) ;
        pass.setFilterLimits(0.0,0.35);
        pass.filter(*cloudfiltered);

        pass.setInputCloud(cloudfiltered) ;
        pass.setFilterFieldName("x" ) ;
        pass.setFilterLimits(-0.17, 0.12);
        pass.filter(*cloudfiltered);

        pass.setInputCloud(cloudfiltered) ;
        pass.setFilterFieldName("y" ) ;
        pass.setFilterLimits(-0.09, 0.2);
        pass.filter(*cloudfiltered);

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloudfiltered2(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
        sor.setInputCloud (cloudfiltered);
        sor.setMeanK (20);
        sor.setStddevMulThresh (0.2);
        sor.filter (*cloudfiltered2);

        std::ostringstream filteredCloud;
        filteredCloud << filePath<<"_f.ply";
        pcl::io::savePLYFileBinary(filteredCloud.str(), *cloudfiltered2);
    }

    void
    point_cloud_smoothing(std::string filePath){

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1 (new pcl::PointCloud<pcl::PointXYZ>);
        std::ostringstream plyPath;
        plyPath << filePath << ".ply";
        pcl::io::loadPLYFile (plyPath.str(), *cloud1);
        //MLS SMOOTHING
        MovingLeastSquares<PointXYZ, PointXYZ> mls;
        mls.setInputCloud (cloud1);
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
        std::ostringstream filteredCloud;
        filteredCloud << filePath<<"_s.ply";
        pcl::io::savePLYFileBinary(filteredCloud.str(), *cloud_smoothed);
    }

    void
    point_cloud_average(std::string filePath, int cloud_num){

        pcl::PointCloud<pcl::PointXYZ>::Ptr avg_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr sum_cloud(new pcl::PointCloud<pcl::PointXYZ>);

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_sum(new pcl::PointCloud<pcl::PointXYZRGB>);

        std::vector< pcl::PointCloud<pcl::PointXYZ>::Ptr> cloud_vector;

        for (int i=0; i<cloud_num; i++){
            std::ostringstream plyPath;
            plyPath << filePath<< i <<"_f.ply";
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
    delete_boundaries(std::string filePath, int proc_num)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        std::ostringstream inFile;
        inFile << filePath <<".ply";
        pcl::io::loadPLYFile(inFile.str(), *in_cloud);
        std::ostringstream outFile;
        outFile << filePath <<"_nb.ply";
        pcl::io::savePLYFileBinary(outFile.str(),*in_cloud);

        for (int i=0;i<proc_num;i++){

            pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::io::loadPLYFile(outFile.str(), *out_cloud);
            pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
            pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
            ne.setInputCloud (out_cloud);
            pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
            ne.setSearchMethod (tree);
            ne.setRadiusSearch (0.002);
            ne.compute (*normals);
            pcl::PointCloud<pcl::Boundary> boundaries;
            pcl::BoundaryEstimation<pcl::PointXYZ, pcl::Normal, pcl::Boundary> est;
            est.setInputCloud (out_cloud);
            est.setInputNormals (normals);
            est.setRadiusSearch (0.002);
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
//            cout<< " "<<i<<" border deleted"<<endl;
        }
    }

    void
    triangulate_cloud(std::string filePath){

        std::ostringstream inFile;
        inFile << filePath <<".ply";
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::io::loadPLYFile (inFile.str().c_str(), *cloud);
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
        std::ostringstream smooth_meshPath;
        smooth_meshPath << filePath<<"_mesh.ply";
        pcl::io::savePolygonFile(smooth_meshPath.str().c_str(), triangles,false);

    }


}