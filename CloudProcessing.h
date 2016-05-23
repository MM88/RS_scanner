//
// Created by miky on 23/05/16.
//

#ifndef RS_SCANNER_CLOUDPROCESSING_H
#define RS_SCANNER_CLOUDPROCESSING_H

using namespace pcl;
using namespace std;

class CloudProcessing {

private:

    std::string txt_path;
    int num_border_to_remove;
    bool filtering = true;
    bool remove_border = true;
    bool smoothing = true;
    std::string outPath;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
    pcl::PolygonMesh output_mesh;

    void point_cloud_maker();
    void point_cloud_filtering();
    void point_cloud_smoothing();
    void delete_boundaries();
    pcl::PolygonMesh triangulate_cloud();

public:

    virtual ~CloudProcessing() { }
    CloudProcessing() {
    }

    void setTxt_path(const string &txt_path) {
        CloudProcessing::txt_path = txt_path;
    }

    void setNum_border_to_remove(int num_border_to_remove) {
        CloudProcessing::num_border_to_remove = num_border_to_remove;
    }

    void setFiltering(bool filtering) {
        CloudProcessing::filtering = filtering;
    }

    void setRemove_border(bool remove_border) {
        CloudProcessing::remove_border = remove_border;
    }

    void setSmoothing(bool smoothing) {
        CloudProcessing::smoothing = smoothing;
    }

    void setOutPath(const string &outPath) {
         CloudProcessing::outPath = outPath;
    }

    void processCloud();

};


#endif //RS_SCANNER_CLOUDPROCESSING_H
