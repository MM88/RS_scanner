#include <pcl/visualization/cloud_viewer.h>
#include <cv.h>
#include <gtkmm.h>
#include <time.h>
#include "CloudProcessing.h"
#include <pcl/io/ply_io.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/sac_model_sphere.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include "rs_grabber.h"

using namespace pcl;
using namespace std;
using namespace cv;
using namespace std;
using namespace Gtk;


void on_scan_button_click()
{
    RSCloudGrabber::grab_clouds();

}

void on_proc_button_click() {

    int cloud_num = RSCloudGrabber::get_dev_number();

    for(int i=0;i<1;i++){

        std::ostringstream inPath;
        inPath << "/home/miky/Scrivania/nuvole/pCloud_" <<i<<".txt";
        std::ostringstream outPath;
        outPath << "/home/miky/Scrivania/nuvole/pCloud_"<<i<<".ply";

        CloudProcessing cp;
        cp.setNum_border_to_remove(3);
        cp.setFiltering(true);
        cp.setSmoothing(true);
        cp.setRemove_border(true);
        cp.setTxt_path(inPath.str().c_str());
        cp.setOutPath(outPath.str().c_str());
        cp.processCloud();

    }
}

void on_calib_button_click(){

}
boost::shared_ptr<pcl::visualization::PCLVisualizer>
simpleVis (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
{
    // --------------------------------------------
    // -----Open 3D viewer and add point cloud-----
    // --------------------------------------------
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (0, 0, 0);
    viewer->addPointCloud<pcl::PointXYZ> (cloud, "sample cloud");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
    //viewer->addCoordinateSystem (1.0, "global");
    viewer->initCameraParameters ();
    return (viewer);
}

int main(int argc, char *argv[])
{

    auto app = Gtk::Application::create();

    Gtk::Window window;
    Gtk::ButtonBox butbox;
    Gtk::Button scan_button("Scan");
    Gtk::Button process_button("Proc clouds");
    Gtk::Button calib_button("Calibrazione");

    window.set_default_size(100, 100);
    window.set_position(Gtk::WIN_POS_CENTER);
    scan_button.signal_clicked().connect(sigc::ptr_fun(&on_scan_button_click));
    process_button.signal_clicked().connect(sigc::ptr_fun(&on_proc_button_click));
    calib_button.signal_clicked().connect(sigc::ptr_fun(&on_calib_button_click));

    scan_button.show();
    process_button.show();
    calib_button.show();
    butbox.add(scan_button);
    butbox.add(process_button);
    butbox.add(calib_button);
    butbox.show();
    window.add(butbox);

    return app->run(window);


    // CODICE PER TROVARE CENTRO SFERE

//    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
//    pcl::io::loadPLYFile("/home/miky/Scrivania/nuvole/cloud_p.ply",*cloud);
//    Eigen::VectorXf sphere_params; //3D center and radius
//
////    pcl::SampleConsensusModelSphere<pcl::PointXYZ>::Ptr sphere_model ( new pcl::SampleConsensusModelSphere<pcl::PointXYZ>(cloud));
////    pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(sphere_model);
////    ransac.setDistanceThreshold(.004);
////    ransac.computeModel();
////    ransac.getModelCoefficients(sphere_params);
//
//    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_sphere (new pcl::PointCloud<pcl::PointXYZ>);
//    cloud->is_dense = true;
//
//
//
//    // RANSAC objects: model and algorithm.
//    pcl::SampleConsensusModelSphere<pcl::PointXYZ>::Ptr model(new pcl::SampleConsensusModelSphere<pcl::PointXYZ>(cloud));
//    pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(model);
//    model->setRadiusLimits(0.03, 0.04);
////    model->setRadiusLimits(0.01, 0.025);
//    ransac.setMaxIterations(10000);
//    ransac.setDistanceThreshold(.004);
//    ransac.computeModel();
//    ransac.getModelCoefficients(sphere_params);
//    std::cout << "Sphere coefficients: " << sphere_params[0] << " " << sphere_params[1] << " " << sphere_params[2] << " " << sphere_params[3] << std::endl;
//
//    std::vector<int> inlierIndices;
//    ransac.getInliers(inlierIndices);
//    if (inlierIndices.size() == 0)
//    {
//        PCL_ERROR ("Could not estimate a sphere model for the given dataset. ");
//        std::cerr << "No sphere found" << std::endl;
//        //return (-1);
//    }
//    // Copy all inliers of the model to another cloud.
//    pcl::copyPointCloud<pcl::PointXYZ>(*cloud, inlierIndices, *cloud_sphere);
//
//    // creates the visualization object and adds either our orignial cloud or all of the inliers
//    // depending on the command line arguments specified.
//    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer1, viewer2;
//
//    viewer1 = simpleVis(cloud_sphere);
//
//    while (!viewer1->wasStopped ())
//    {
//        viewer1->spinOnce (100);
//        boost::this_thread::sleep (boost::posix_time::microseconds (100000));
//    }

//




}

