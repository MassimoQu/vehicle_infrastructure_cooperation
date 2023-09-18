#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include <pcl/common/common.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/crop_box.h>

// #include <opencv2/opencv.hpp>

#include <Eigen/Dense>
#include <Eigen/Core>

#include <jsoncpp/json/json.h>

#include <iostream>


// pcl中筛选离群点的内嵌算法

// 通过 个体点云点留存率 统计 整体点云留存率 的指标

// 根据不同的类别给框上不同的颜色，观察真值的每一类对象本身的提取效果（后期效果不好的一个调整思路


struct BOX2D{
    double xmin, ymin, xmax, ymax;
    BOX2D(double xmin, double ymin, double xmax, double ymax) : xmin(xmin), ymin(ymin), xmax(xmax), ymax(ymax) {}
    BOX2D() : xmin(0), ymin(0), xmax(0), ymax(0) {}
};
struct BOX3D{
    double yaw;
    struct {
        double h, w, l;
    } dimensions;
    struct {
        double x, y, z;
    } location;

    BOX3D(double yaw, double h, double w, double l, double x, double y, double z) : yaw(yaw), dimensions{h, w, l}, location{x, y, z} {}
    BOX3D() : yaw(0), dimensions{0, 0, 0}, location{0, 0, 0} {}
};
std::istream& operator>>(std::istream& is, BOX2D& b) {
    is >> b.xmin >> b.ymin >> b.xmax >> b.ymax;
    return is;
}
std::istream& operator>>(std::istream& is, BOX3D& b) {
    is >> b.yaw >> b.dimensions.h >> b.dimensions.w >> b.dimensions.l
       >> b.location.x >> b.location.y >> b.location.z;
    return is;
}


bool read_point_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr result_point_cloud, std::string file_name){

    if(pcl::io::loadPCDFile<pcl::PointXYZ>(file_name, * result_point_cloud) == -1)
    {    
        PCL_ERROR("Couldn't read file test_pcd.pcd \n");
        return  false;
    }
    std::cout << "read point cloud success" << endl;
    return true;

}

bool read_data(pcl::PointCloud<pcl::PointXYZ>::Ptr result_point_cloud, std::string point_cloud_file_name,
                std::vector<BOX2D> &box2d_scene, std::string file_name_2d, std::vector<BOX3D> &box3d_scene, std::string file_name_3d){
    
    bool result = true;

    result = result && read_point_cloud(result_point_cloud, point_cloud_file_name);

    // 优雅的读取json数据并赋值给box2d和box3d 要重载from_json 并引入 nlohmann json 解析库 #unnecessary

    Json::Reader reader;

    std::ifstream ifs_2d;
    ifs_2d.open(file_name_2d); 
    assert(ifs_2d.is_open());
    Json::Value json_tree_2d;
    if (!reader.parse(ifs_2d, json_tree_2d, false))
    {
        cerr << "parse 2dboxing failed \n";
        return false;
    }
    
    for (unsigned int i = 0; i < json_tree_2d.size(); i++) {
        box2d_scene.push_back(BOX2D(json_tree_2d[i]["2d_box"]["xmin"].asDouble(), json_tree_2d[i]["2d_box"]["ymin"].asDouble(), 
                                    json_tree_2d[i]["2d_box"]["xmax"].asDouble(), json_tree_2d[i]["2d_box"]["ymax"].asDouble()));
       
    }
    ifs_2d.close();

    std::ifstream ifs_3d;
    ifs_3d.open(file_name_3d); 
    assert(ifs_3d.is_open());
    Json::Value json_tree_3d;
    if (!reader.parse(ifs_3d, json_tree_3d, false))
    {
        cerr << "parse 3dboxing failed \n";
        return false;
    }
    for (unsigned int i = 0; i < json_tree_3d.size(); i++) {
        box3d_scene.push_back(BOX3D(json_tree_3d[i]["yaw"].asDouble(), json_tree_3d[i]["3d_dimensions"]["h"].asDouble(), json_tree_3d[i]["3d_dimensions"]["w"].asDouble(), json_tree_3d[i]["3d_dimensions"]["l"].asDouble(), 
                                    json_tree_3d[i]["3d_location"]["x"].asDouble(), json_tree_3d[i]["3d_location"]["y"].asDouble(), json_tree_3d[i]["3d_location"]["z"].asDouble()));

    }
    ifs_3d.close();

    return true;
}

void show_boxed_target_with_color(pcl::PointCloud<pcl::PointXYZ>::Ptr scene_point_cloud, std::vector<BOX3D> box3d_scene){

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr scene_point_cloud_rgb (new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::copyPointCloud(*scene_point_cloud, *scene_point_cloud_rgb);
     // set scene_point_cloud_rgb color
    for (int i = 0; i < scene_point_cloud_rgb->size(); i++) {
        scene_point_cloud_rgb->points[i].r = 0;
        scene_point_cloud_rgb->points[i].g = 255;
        scene_point_cloud_rgb->points[i].b = 0;
    }

    pcl::PointIndices::Ptr inliers(new pcl::PointIndices()), outliers(new pcl::PointIndices());

    for (int i = 0; i < box3d_scene.size(); i++) { 

        int xmin = box3d_scene[i].location.x - box3d_scene[i].dimensions.l / 2, ymin = box3d_scene[i].location.y - box3d_scene[i].dimensions.w / 2, zmin = box3d_scene[i].location.z - box3d_scene[i].dimensions.h / 2, 
            xmax = box3d_scene[i].location.x + box3d_scene[i].dimensions.l / 2, ymax = box3d_scene[i].location.y + box3d_scene[i].dimensions.w / 2, zmax = box3d_scene[i].location.z + box3d_scene[i].dimensions.h / 2;

        pcl::CropBox<pcl::PointXYZRGB> crop_box_filter;
        crop_box_filter.setInputCloud(scene_point_cloud_rgb);
        crop_box_filter.setMin(Eigen::Vector4f(xmin, ymin, zmin, 1.0));
        crop_box_filter.setMax(Eigen::Vector4f(xmax, ymax, zmax, 1.0));
        crop_box_filter.setNegative(false);//默认false，保留box内的点
        // rotate crop_box using crop_box_filter.setRotation
        // crop_box_filter.setRotation(Eigen::Vector3f(0, 0, -box3d_scene[i].yaw)); 
        // verify the effectiveness of the rotation #todo
        // above shows the idea of rotation of the box , another idea is to rotate the point cloud #todo
        
        crop_box_filter.filter(inliers->indices);
        for(int i = 0; i < inliers->indices.size(); i++){
            // scene_point_cloud_rgb->points[inliers->indices[i]].r = 255;
            // scene_point_cloud_rgb->points[inliers->indices[i]].g = 0;
            // scene_point_cloud_rgb->points[inliers->indices[i]].b = 0;

            auto& point = scene_point_cloud_rgb->at(inliers->indices.at(i));
            point.r = 255;
            point.g = 0;
            point.b = 0;
        }

        // boxed target
        // viewer.addCube(xmin, xmax, ymin, ymax, zmin, zmax, 1, 0, 0, "box3d_" + std::to_string(i));

        // cout indices.size
        std::cout << "box3d_" << i << "(inner)_indices: " << inliers->indices.size() << endl;
        std::cout << "box3d_" << i << "(outer): " << scene_point_cloud_rgb->size() - inliers->indices.size() << endl;

        // delete boxed target as loop progresses (proven to be unnecessary)
        // pcl::PointCloud<pcl::PointXYZRGB>::Ptr scene_point_cloud_without_boxx(new pcl::PointCloud<pcl::PointXYZRGB>);
        // crop_box_filter.setNegative(true);//true过滤box内的点
        // crop_box_filter.filter(*scene_point_cloud_without_boxx);
        // crop_box_filter.filter(outliers->indices);
        // for (int i = 0; i < outliers->indices.size(); i++) {
        //     auto& point = scene_point_cloud_rgb->at(outliers->indices.at(i));
        //     point.r = 0;
        //     point.g = 0;
        //     point.b = 255;
        // }
        // // 用pcl 的方法 使得 scene_point_cloud_without_boxx 代替 scene_point_cloud #done
        // scene_point_cloud_without_boxx.swap(scene_point_cloud_rgb);


    }

    pcl::visualization::PCLVisualizer viewer("scene_point_cloud_rgb");
    viewer.setBackgroundColor(0.0, 0.0, 0.0);
    viewer.addCoordinateSystem(1.0);

    viewer.addPointCloud<pcl::PointXYZRGB> (scene_point_cloud_rgb, "scene_point_cloud_rgb");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "scene_point_cloud_rgb");
   
    viewer.addCoordinateSystem (1.0);
    viewer.initCameraParameters(); // 啥作用？

    while (!viewer.wasStopped())
    {
        // viewer.spinOnce();
        viewer.spinOnce(100);
        // std::this_thread::sleep_for(100ms);
    }

    return;
}

int main(){
    pcl::PointCloud<pcl::PointXYZ>::Ptr scene_point_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    std::vector<BOX2D> box2d_scene;
    std::vector<BOX3D> box3d_scene;

    read_data(scene_point_cloud, "../cooperative-vehicle-infrastructure-example_12649705822621696/cooperative-vehicle-infrastructure-example/infrastructure-side/velodyne/000010.pcd", 
        box2d_scene, "../cooperative-vehicle-infrastructure-example_12649705822621696/cooperative-vehicle-infrastructure-example/infrastructure-side/label/camera/000010.json", 
        box3d_scene, "../cooperative-vehicle-infrastructure-example_12649705822621696/cooperative-vehicle-infrastructure-example/infrastructure-side/label/virtuallidar/000010.json"
        );
    
    std::cout << "scene_point_cloud size: " << scene_point_cloud->size() << std::endl;
    std::cout << "box2d_scene size: " << box2d_scene.size() << std::endl;
    std::cout << "box3d_scene size: " << box3d_scene.size() << std::endl;

    // for (int i = 0; i < box2d_scene.size(); i++) {
    //     std::string name_2d = "box2d_" + std::to_string(i);
    //     viewer.addCube(box2d_scene[i].xmin, box2d_scene[i].xmax, box2d_scene[i].ymin, box2d_scene[i].ymax, 0, 0, 1, 0, 0, name_2d);
    // }

    show_boxed_target_with_color(scene_point_cloud, box3d_scene);
    // 框框和点云目标并不完全重合，看起来这个json的检测框不是针对这个点云的？#todo  


    /** visualize 
    
    pcl::visualization::PCLVisualizer viewer("scene_point_cloud");
    viewer.setBackgroundColor(0.0, 0.0, 0.0);
    viewer.addCoordinateSystem(1.0);

    viewer.addPointCloud<pcl::PointXYZ> (scene_point_cloud, "scene_point_cloud");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "scene_point_cloud");
   
    viewer.addCoordinateSystem (1.0);
    viewer.initCameraParameters(); // 啥作用？

    while (!viewer.wasStopped())
    {
        // viewer.spinOnce();
        viewer.spinOnce(100);
        // std::this_thread::sleep_for(100ms);
    }

    **/
    


    return  0;
}
