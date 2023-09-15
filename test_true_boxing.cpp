#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include <pcl/common/common.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>

// #include <opencv2/opencv.hpp>

#include <Eigen/Dense>
#include <Eigen/Core>

#include <jsoncpp/json/json.h>

#include <iostream>


// boxing 用什么数据结构

// 从json文件中读取boxing数据

// 点云展示方法

// 读取点云

// pcl中筛选离群点的内嵌算法

// 通过 个体点云点留存率 统计 整体点云留存率 的指标




struct BOX2D{
    double xmin, ymin, xmax, ymax;
    BOX2D(double xmin, double ymin, double xmax, double ymax) : xmin(xmin), ymin(ymin), xmax(xmax), ymax(ymax) {}
    BOX2D() : xmin(0), ymin(0), xmax(0), ymax(0) {}
};
struct BOX3D{
    double yaw;
    struct {
        float h, w, l;
    } dimensions;
    struct {
        float x, y, z;
    } location;

    BOX3D(double yaw, float h, float w, float l, float x, float y, float z) : yaw(yaw), dimensions{h, w, l}, location{x, y, z} {}
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

    // 优雅的读取json数据并赋值给box2d和box3d 要重载from_json 并引入 nlohmann json 解析库 #todo

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
        box2d_scene.push_back(BOX2D(json_tree_2d[i]["xmin"].asDouble(), json_tree_2d[i]["ymin"].asDouble(), json_tree_2d[i]["xmax"].asDouble(), json_tree_2d[i]["ymax"].asDouble()));
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
        box3d_scene.push_back(BOX3D(json_tree_2d[i]["yaw"].asDouble(), 
                json_tree_2d[i]["dimensions"]["h"].asFloat(), json_tree_2d[i]["dimensions"]["w"].asFloat(), json_tree_2d[i]["dimensions"]["l"].asFloat(), 
                json_tree_2d[i]["location"]["x"].asFloat(), json_tree_2d[i]["location"]["y"].asFloat(), json_tree_2d[i]["location"]["z"].asFloat()));
    }
    ifs_3d.close();

    return true;
}

// void process(pcl::PointCloud<pcl::PointXYZI>::Ptr object_point_cloud, Eigen::Matrix4f RT, Eigen::Matrix3f camera_param)
// {

//     Eigen::Matrix<float, 3, 4> T_lidar2cam_top3_local, T_lidar2image_local; //lida2image=T_lidar2cam*(T_cam02cam2)*T_cam2image
//     T_lidar2cam_top3_local = RT.topRows(3);                                 
//     T_lidar2image_local = camera_param * T_lidar2cam_top3_local;
//     Eigen::Vector4f raw_point;
//     Eigen::Vector3f trans_point;
//     int point_r = 2;
//     for (int i = 0; i < object_point_cloud->size(); i++)
//     {
//         raw_point(0, 0) = object_point_cloud->points[i].x; 
//         raw_point(1, 0) = object_point_cloud->points[i].y;
//         raw_point(2, 0) = object_point_cloud->points[i].z;
//         raw_point(3, 0) = 1;
//         trans_point = T_lidar2image_local * raw_point;
//         int x = (int)(trans_point(0, 0) / trans_point(2, 0));
//         int y = (int)(trans_point(1, 0) / trans_point(2, 0));
//         // 2d boxing filtration
//     }
// }



// int main(int argc, char **argv) {
    
//     read();

//     double mean_remained_ratio = 0;

//     for(){
//         component_point_cloud = get_component_point_cloud(scene_point_cloud, boxing_3d);
//         double component_remained_ratio = calculate_3d_throught_2d_remaining(component_point_cloud, boxing_2d);
        
//         // metrics of integration
//         mean_remained_ratio += component_remained_ratio;

//     }


//     return 0;
// }

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

    // 点云展示方法




    // show all the box2d and box3d
    pcl::visualization::PCLVisualizer viewer("scene_point_cloud");
    viewer.setBackgroundColor(0.0, 0.0, 0.0);
    

    // viewer.addCoordinateSystem(1.0);

    viewer.addPointCloud<pcl::PointXYZ> (scene_point_cloud, "scene_point_cloud");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "scene_point_cloud");
    
    viewer.addCoordinateSystem (1.0);
    viewer.initCameraParameters(); // 啥作用？

    // for (int i = 0; i < box2d_scene.size(); i++) {
    //     std::string name_2d = "box2d_" + std::to_string(i);
    //     viewer.addCube(box2d_scene[i].xmin, box2d_scene[i].xmax, box2d_scene[i].ymin, box2d_scene[i].ymax, 0, 0, 1, 0, 0, name_2d);
    // }
    // for (int i = 0; i < box3d_scene.size(); i++) {
    //     std::string name_3d = "box3d_" + std::to_string(i);
    //     viewer.addCube(box3d_scene[i].location.x - box3d_scene[i].dimensions.l / 2, box3d_scene[i].location.x + box3d_scene[i].dimensions.l / 2, 
    //         box3d_scene[i].location.y - box3d_scene[i].dimensions.w / 2, box3d_scene[i].location.y + box3d_scene[i].dimensions.w / 2, 
    //         box3d_scene[i].location.z - box3d_scene[i].dimensions.h / 2, box3d_scene[i].location.z + box3d_scene[i].dimensions.h / 2, 1, 0, 0, name_3d);
    // }

    // example1:
    // pcl::visualization::CloudViewer viewer("cloudviewer test");
    // viewer.showCloud(scene_point_cloud);

    // example2:
    // pcl::visualization::PCLVisualizer viewer("PCL Viewer");
    // viewer.setBackgroundColor(0.0, 0.0, 0.0);
    // pcl::PointXYZ o;
    // o.x = 1.0;
    // o.y = 0;
    // o.z = 0;
    // viewer.addSphere(o, 0.25, "sphere", 0);

    // example3:
    // pcl::visualization::PCLVisualizer viewer("PCL Viewer");
    // viewer.setBackgroundColor(1, 1, 1);


    while (!viewer.wasStopped())
    {
        // viewer.spinOnce();
        viewer.spinOnce(100);
        // std::this_thread::sleep_for(100ms);
    }

    cout<<"new one"<<endl;

    return  0;
}
