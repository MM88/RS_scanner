#include <pcl-1.8/pcl/visualization/cloud_viewer.h>
#include <cv.h>
#include "cloud_utils.h"
#include <librealsense/rs.hpp>
#include <cstdio>
#include <fstream>
#include <boost/thread.hpp>
#include <iostream>
#include <string>

using namespace pcl;
using namespace std;
using namespace cv;
using namespace CloudUtils;
using namespace rs;
using namespace std;

void scan (){

    // Turn on logging. We can separately enable logging to console or to file, and use different severity filters for each.
    rs::log_to_console(rs::log_severity::warn);
    //rs::log_to_file(rs::log_severity::debug, "librealsense.log");

    // Create a context object. This object owns the handles to all connected realsense devices.
    rs::context ctx;
    printf("There are %d connected RealSense devices.\n", ctx.get_device_count());
    if(ctx.get_device_count() == 0) return;

    // Enumerate all devices
    std::vector<rs::device *> devices;
    for(int i=0; i<ctx.get_device_count(); ++i)
    {
        devices.push_back(ctx.get_device(i));
    }

    // Configure and start our devices
    for(auto dev : devices)
    {
        std::cout << "Starting " << dev->get_name() << "... ";
        dev->enable_stream(rs::stream::depth, rs::preset::best_quality);
        dev->enable_stream(rs::stream::color, rs::preset::best_quality);
        dev->start();
        dev->set_option((rs::option)12, (double)0); //laser power //def 16
        dev->set_option((rs::option)13, (double)1);  //accuracy //def 2 meglio mesh butterata ma senza ondine
        dev->set_option((rs::option)15, (double)5); //filter option  (1 3 4 ) //def 5 Moderate smoothing effect optimized for distances between 550mm to 850mm for F200 to balance between good sharpness level, high accuracy and moderate noise artifacts.

        std::cout << "done." << std::endl;
    }

    // This tutorial will access only a single device, but it is trivial to extend to multiple devices

    for(auto dev : devices)
    {

        dev->set_option((rs::option)12, (double)16); //laser power //def 16

        // Capture frames to give autoexposure
        for (int i = 0; i < 30; ++i) dev->wait_for_frames();

        dev->wait_for_frames();

        std::ofstream myfile;

        int fileno = 0;
        bool success;
        std::string fileName = "/home/miky/Scrivania/nuvole/pCloud_" + std::to_string(fileno)  + ".txt";
        std::ofstream ifs(fileName, std::ios::in | std::ios::ate);
        success = ifs.is_open();
        while(success) {
            fileno++;//increase by one to get a new file name
            fileName.clear();
            fileName = "/home/miky/Scrivania/nuvole/pCloud_" + std::to_string(fileno)  + ".txt";
            std::ofstream ifs(fileName, std::ios::in | std::ios::ate);
            success = ifs.is_open();
        }
        myfile.open(fileName, std::ios::app);

        // Retrieve our images
        const uint16_t * depth_image = (const uint16_t *)dev->get_frame_data(rs::stream::depth);
        const uint8_t * color_image = (const uint8_t *)dev->get_frame_data(rs::stream::color);

        // Retrieve camera parameters for mapping between depth and color
        rs::intrinsics depth_intrin = dev->get_stream_intrinsics(rs::stream::depth);
        rs::extrinsics depth_to_color = dev->get_extrinsics(rs::stream::depth, rs::stream::color);
        rs::intrinsics color_intrin = dev->get_stream_intrinsics(rs::stream::color);
        float scale = dev->get_depth_scale();
        std::cout<<scale<<std::endl;

        for(int dy=0; dy<depth_intrin.height; ++dy)
        {
            for(int dx=0; dx<depth_intrin.width; ++dx)
            {
                // Retrieve the 16-bit depth value and map it into a depth in meters
                uint16_t depth_value = depth_image[dy * depth_intrin.width + dx];
                float depth_in_meters = depth_value * scale; //0.1*3.12352;// scale*10000/3.12352;

                // Skip over pixels with a depth value of zero, which is used to indicate no data
                if(depth_value == 0) continue;

                // Map from pixel coordinates in the depth image to pixel coordinates in the color image
                rs::float2 depth_pixel = {(float)dx, (float)dy};
                rs::float3 depth_point = depth_intrin.deproject(depth_pixel, depth_in_meters);
                rs::float3 color_point = depth_to_color.transform(depth_point);
                rs::float2 color_pixel = color_intrin.project(color_point);

                myfile << depth_point.x << '\t' << depth_point.y << '\t' << depth_point.z << std::endl;

            }
        }
        myfile.close();
        dev->set_option((rs::option)12, (double)0);
//        boost::this_thread::sleep (boost::posix_time::seconds (5));

    }
}
int main() {

    scan();


    /* delete borders */
//    int proc_num = 3;
//    std::string filePath = "/home/miky/Scrivania/pCloud_0_filtered";
//    CloudUtils::compute_boundaries (filePath, proc_num);

    /* compute cloud resolution */
//    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
//    std::ostringstream sumCloud;
//    sumCloud <<"/home/miky/Scrivania/pCloud_0.ply";
//    pcl::io::loadPLYFile(sumCloud.str(), *cloud);
//    cout<<CloudUtils::computeCloudResolution(cloud);

    /* make ply from txt and filter */
//    for (int i=0;i<2;i++){
//        std::ostringstream filePath;
//        filePath << "/home/miky/Scrivania/nuvole/pCloud_"<<i;
//        CloudUtils::point_cloud_maker(filePath.str());
//        CloudUtils::point_cloud_filtering(filePath.str());
//    }


    return 0;
}