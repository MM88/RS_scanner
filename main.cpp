#include <pcl-1.8/pcl/visualization/cloud_viewer.h>
#include <cv.h>
#include "cloud_utils.h"
#include <librealsense/rs.hpp>
#include <gtkmm.h>
#include <time.h>
#include "CloudProcessing.h"

using namespace pcl;
using namespace std;
using namespace cv;
using namespace CloudUtils;
using namespace rs;
using namespace std;
using namespace Gtk;

int dev_number;

void on_scan_button_click()
{
    // Turn on logging. We can separately enable logging to console or to file, and use different severity filters for each.
    rs::log_to_console(rs::log_severity::warn);
    //rs::log_to_file(rs::log_severity::debug, "librealsense.log");

    // Create a context object. This object owns the handles to all connected realsense devices.
    rs::context ctx;
    printf("There are %d connected RealSense devices.\n", ctx.get_device_count());
    if(ctx.get_device_count() == 0) return;

    dev_number = ctx.get_device_count();

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
        dev->set_option((rs::option)12, (double)0); //laser power
        dev->set_option((rs::option)13, (double)1);  //accuracy
        dev->set_option((rs::option)15, (double)5); //filter option
        std::cout << "done." << std::endl;
    }

    // This tutorial will access only a single device, but it is trivial to extend to multiple devices
    std::vector<std::vector<rs::float3>> RSclouds;

    const clock_t begin_time = clock(); //to check grabbing time

    for(auto dev : devices)
    {
        dev->set_option((rs::option)12, (double)16);

        // Capture frames to give autoexposure
        for (int i = 0; i < 15; ++i) dev->wait_for_frames();

        dev->wait_for_frames();
        // Retrieve our images
        const uint16_t * depth_image = (const uint16_t *)dev->get_frame_data(rs::stream::depth);
        const uint8_t * color_image = (const uint8_t *)dev->get_frame_data(rs::stream::color);

        // Retrieve camera parameters for mapping between depth and color
        rs::intrinsics depth_intrin = dev->get_stream_intrinsics(rs::stream::depth);
        rs::extrinsics depth_to_color = dev->get_extrinsics(rs::stream::depth, rs::stream::color);
        rs::intrinsics color_intrin = dev->get_stream_intrinsics(rs::stream::color);
        float scale = dev->get_depth_scale();

        std::vector<rs::float3> point_cloud;
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

                point_cloud.push_back(depth_point);

            }
        }
        RSclouds.push_back(point_cloud);

        dev->set_option((rs::option)12, (double)0);
//        boost::this_thread::sleep (boost::posix_time::seconds (5));
    }
    std::cout << float( clock () - begin_time ) /  CLOCKS_PER_SEC<<endl;
    std::cout << "done scanning." << std::endl;
    std::cout << "saving clouds.." << std::endl;

    for (int i=0;i<devices.size();i++){
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
        for (int j=0;j<RSclouds[i].size();j++)
            myfile << RSclouds[i][j].x << '\t' << RSclouds[i][j].y << '\t' << RSclouds[i][j].z << std::endl;
        myfile.close();
    }

    std::cout << "done saving." << std::endl;

    return;

}
void on_proc_button_click() {

    int cloud_num = 2; //dev_number;

    for(int i=0;i<cloud_num;i++){

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
//    int cloud_num = 1; //= dev_numver; // num of cameras/cloud to grab
//    int proc_num = 3; // times of border processing
//
//    for(int i=0;i<cloud_num;i++){
//            /* make ply from txt and filter */
//            std::ostringstream filePath;
//            filePath << "/home/miky/Scrivania/nuvole/pCloud_"<<i;
//            CloudUtils::point_cloud_maker(filePath.str());
//            CloudUtils::point_cloud_filtering(filePath.str());
//            /* delete borders */
//            filePath.str("");
//            filePath <<"/home/miky/Scrivania/nuvole/pCloud_"<<i<<"_f";
//            CloudUtils::delete_boundaries(filePath.str(), proc_num);
//            /* smooth points */
//            filePath.str("");
//            filePath <<"/home/miky/Scrivania/nuvole/pCloud_"<<i<<"_f_nb";
//            CloudUtils::point_cloud_smoothing(filePath.str());
//            /* create mesh */
//            filePath.str("");
//            filePath <<"/home/miky/Scrivania/nuvole/pCloud_"<<i<<"_f_nb_s";
//            CloudUtils::triangulate_cloud(filePath.str());
//    }
//


}

int main(int argc, char *argv[])
{
    auto app = Gtk::Application::create( "org.gtkmm.examples.base");

    Gtk::Window window;
    Gtk::ButtonBox butbox;
    Gtk::Button scan_button("Scan");
    Gtk::Button process_button("Proc clouds");

    window.set_default_size(100, 100);
    window.set_position(Gtk::WIN_POS_CENTER);
    scan_button.signal_clicked().connect(sigc::ptr_fun(&on_scan_button_click));
    process_button.signal_clicked().connect(sigc::ptr_fun(&on_proc_button_click));

    scan_button.show();
    process_button.show();
    butbox.add(scan_button);
    butbox.add(process_button);
    butbox.show();
    window.add(butbox);

    return app->run(window);
}