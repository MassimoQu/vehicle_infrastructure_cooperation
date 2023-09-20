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

bool read_data( //cv::Mat &image, std::string image_file_name, 
                pcl::PointCloud<pcl::PointXYZ>::Ptr result_point_cloud, std::string point_cloud_file_name,
                std::vector<BOX2D> &box2d_scene, std::string file_name_2d, std::vector<BOX3D> &box3d_scene, std::string file_name_3d,
                Eigen::Matrix4f &external_para, std::string external_para_file_name, Eigen::Matrix3f &internal_para, std::string internal_para_file_name                
                ){
    
    bool result = true;

    result = result && read_point_cloud(result_point_cloud, point_cloud_file_name);

    // image = cv::imread(image_file_name);

    // 优雅的读取json数据并赋值给box2d和box3d 要重载from_json 并引入 nlohmann json 解析库 #unnecessary
    // 更优雅的读入实现方式：构造具体数据这一块传递回调函数，把json的读取层做一个抽象提取 #todo #unnecessary

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
        box3d_scene.push_back(BOX3D(json_tree_3d[i]["rotation"].asDouble(), json_tree_3d[i]["3d_dimensions"]["h"].asDouble(), json_tree_3d[i]["3d_dimensions"]["w"].asDouble(), json_tree_3d[i]["3d_dimensions"]["l"].asDouble(), 
                                    json_tree_3d[i]["3d_location"]["x"].asDouble(), json_tree_3d[i]["3d_location"]["y"].asDouble(), json_tree_3d[i]["3d_location"]["z"].asDouble()));

    }
    ifs_3d.close();

    std::ifstream ifs_inter;
    ifs_inter.open(internal_para_file_name); 
    assert(ifs_inter.is_open());
    Json::Value json_tree_inter;
    if (!reader.parse(ifs_inter, json_tree_inter, false))
    {
        cerr << "parse internal failed \n";
        return false;
    }
    // input internal_para from json_tree_inter
    for(unsigned int i = 0; i < json_tree_inter["cam_K"].size(); i++){
        internal_para(i / 3, i % 3) = json_tree_inter["cam_K"][i].asDouble();
    }
    ifs_inter.close();

    std::ifstream ifs_external;
    ifs_external.open(external_para_file_name); 
    assert(ifs_external.is_open());
    Json::Value json_tree_external;
    if (!reader.parse(ifs_external, json_tree_external, false))
    {
        cerr << "parse external failed \n";
        return false;
    }
    // external_para
    for(unsigned int i = 0; i < 3; i++){
        for(unsigned int j = 0; j < 3; j++){
            external_para(i, j) = json_tree_external["rotation"][i][j].asDouble();
        }
        external_para(i, 3) = json_tree_external["translation"][i][0].asDouble();
    }
    external_para(3, 3) = 1;
    ifs_external.close();

    return result;
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

    int total_point_num = scene_point_cloud_rgb->size();

    pcl::visualization::PCLVisualizer viewer("scene_point_cloud_rgb");
    viewer.setBackgroundColor(0.0, 0.0, 0.0);
    viewer.addCoordinateSystem(1.0);

    for (int i = 0; i < box3d_scene.size(); i++) { 

        int xmin = box3d_scene[i].location.x - box3d_scene[i].dimensions.l / 2, ymin = box3d_scene[i].location.y - box3d_scene[i].dimensions.w / 2, zmin = box3d_scene[i].location.z - box3d_scene[i].dimensions.h / 2, 
            xmax = box3d_scene[i].location.x + box3d_scene[i].dimensions.l / 2, ymax = box3d_scene[i].location.y + box3d_scene[i].dimensions.w / 2, zmax = box3d_scene[i].location.z + box3d_scene[i].dimensions.h / 2;

        pcl::CropBox<pcl::PointXYZRGB> crop_box_filter;
        crop_box_filter.setInputCloud(scene_point_cloud_rgb);
        // crop_box_filter.setMin(Eigen::Vector4f(xmin, ymin, zmin, 1.0));
        // crop_box_filter.setMax(Eigen::Vector4f(xmax, ymax, zmax, 1.0));
        crop_box_filter.setMin(Eigen::Vector4f(-box3d_scene[i].dimensions.l / 2, -box3d_scene[i].dimensions.w / 2, -box3d_scene[i].dimensions.h / 2, 1.0));
        crop_box_filter.setMax(Eigen::Vector4f(box3d_scene[i].dimensions.l / 2, box3d_scene[i].dimensions.w / 2, box3d_scene[i].dimensions.h / 2, 1.0));
        crop_box_filter.setNegative(false);//默认false，保留box内的点

        // Eigen::AngleAxisd rotationVector(box3d_scene[i].yaw, Eigen::Vector3d(0, 0, 1));
        // Eigen::Matrix3d rotationMatrix(rotationVector.toRotationMatrix());
        // crop_box_filter.setRotation(rotationMatrix)  ;


        // double angle = box3d_scene[i].yaw;
        // Eigen::Matrix4f transform_matrix;
        // transform_matrix.setZero();
        // transform_matrix(0, 0) = cos(angle);
        // transform_matrix(0, 1) = -sin(angle);
        // transform_matrix(1, 0) = sin(angle);
        // transform_matrix(1, 1) = cos(angle);
        // transform_matrix(2, 2) = 1;
        // transform_matrix(3, 3) = 1;
        // std::cout << "rotation_matrix*****\n" << transform_matrix << endl << endl;
        // crop_box_filter.setTransform(Eigen::Affine3f(transform_matrix));


        // rotate crop_box using crop_box_filter.setRotation
        crop_box_filter.setTranslation(Eigen::Vector3f(box3d_scene[i].location.x, box3d_scene[i].location.y, box3d_scene[i].location.z));
        crop_box_filter.setRotation(Eigen::Vector3f(0.0f, 0.0f, (float) (box3d_scene[i].yaw * M_PI / 180)));  
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

        total_point_num = total_point_num - inliers->indices.size();

        // cout indices.size
        std::cout << "box3d_" << i << "(inner)_indices: " << inliers->indices.size() << endl;
        std::cout << "box3d_" << i << "(outer): " << total_point_num << endl;
        //cout rotation
        std::cout << "box3d_" << i << "(rotation): " << box3d_scene[i].yaw << endl;

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

void calculate_3dboxed_point_cloud_retain2d_rate(pcl::PointCloud<pcl::PointXYZ>::Ptr scene_point_cloud, std::vector<BOX3D> box3d_scene, std::vector<BOX2D> box2d_scene, Eigen::Matrix4f external_para, Eigen::Matrix3f internal_para){


}

// void project2image(pcl::PointCloud<pcl::PointXYZI>::Ptr pc, cv::Mat raw_image, cv::Mat &output_image, Eigen::Matrix4f RT, Eigen::Matrix3f camera_param)
// {
//     Eigen::Matrix<float, 3, 4> T_lidar2cam_top3_local, T_lidar2image_local; //lida2image=T_lidar2cam*(T_cam02cam2)*T_cam2image
//     T_lidar2cam_top3_local = RT.topRows(3);
//     T_lidar2image_local = camera_param * T_lidar2cam_top3_local;
//     if (raw_image.channels() < 3 && raw_image.channels() >= 1)
//     {
//         // std::cout << "before cvt" << std::endl;
//         // cv::cvtColor(raw_image, output_image, cv::COLOR_GRAY2BGR);
//         cv::Mat output_image_3channels(raw_image.rows, raw_image.cols, CV_8UC3, cv::Scalar::all(0));
//         for (int i = 0; i < raw_image.cols; i++)//THis is 2_channel
//         {
//             for (int j = 0; j < raw_image.rows; j++)
//             {
//                 output_image_3channels.at<cv::Vec3b>(j, i)[2] = output_image_3channels.at<cv::Vec3b>(j, i)[1] =
//                 output_image_3channels.at<cv::Vec3b>(j, i)[0] = (int)raw_image.at<uchar>(j, i);
//                 //  (int) raw_image.at<uchar>(j, i);
//                 //�Ҷ�ͼ����
//                 // output_image_3channels.at<cv::Vec3b>(j,i)[0] = (int) raw_image.at<uchar>(j, i);
//             }
//         }
//         output_image_3channels.copyTo(output_image);
//         // cv::namedWindow("cvt", CV_WINDOW_NORMAL);
//         // cv::imshow("cvt", output_image);
//         // cv::waitKey(0);
//         // std::cout << "after cvt" << std::endl;
//     }
//     else
//     {
//         raw_image.copyTo(output_image);//Colorful Picture
//     }
//     pcl::PointXYZI r;
//     Eigen::Vector4f raw_point;
//     Eigen::Vector3f trans_point;
//     double deep, deep_config; //deep_config: normalize, max deep
//     int point_r;
//     deep_config = 80;
//     point_r = 2;
//     //std::cout << "image size; " << raw_image.cols << " * " << raw_image.rows << std::endl;
//     for (int i = 0; i < pc->size(); i++)
//     {
//         r = pc->points[i];
//         raw_point(0, 0) = r.x;
//         raw_point(1, 0) = r.y;
//         raw_point(2, 0) = r.z;
//         raw_point(3, 0) = 1;
//         trans_point = T_lidar2image_local * raw_point;          //ͶӰ����
//         int x = (int)(trans_point(0, 0) / trans_point(2, 0));   //r.x/r.z
//         int y = (int)(trans_point(1, 0) / trans_point(2, 0));   //r.y/r.z
//  //int x = (int)(trans_point(0, 0) / trans_point(2, 0)* T_lidar2image_local);   //r.x/r.z �����ҳ�һ������
//         //cout<<"!!!@@@####"<<x<<" "<<y<<" ";
//         if (x < 0 || x > (raw_image.cols - 1) || y < 0 || y > (raw_image.rows - 1) || trans_point(2, 0) <0 )
//             continue;
//         deep = trans_point(2, 0) / deep_config;     //deep = r.z/deep_config
//         //deep = r.intensity / deep_config;
//         int blue, red, green;
//         if (deep <= 0.5)
//         {
//             green = (int)((0.5 - deep) / 0.5 * 255);
//             red = (int)(deep / 0.5 * 255);
//             blue = 0;
//         }
//         else if (deep <= 1)
//         {
//             green = 0;
//             red = (int)((1 - deep) / 0.5 * 255);
//             blue = (int)((deep - 0.5) / 0.5 * 255);
//         }
//         else
//         {
//             blue = 0;
//             green = 0;
//             red = 255;
//         };
//         cv::circle(output_image, cv::Point2f(x, y), 4, cv::Scalar(0, 255, 0), -1);
//         // cv::circle(output_image, cv::Point2f(x, y), point_r, cv::Scalar(blue,green,red), -1);
//     }
// }

int main(){
    // cv::Mat image;
    pcl::PointCloud<pcl::PointXYZ>::Ptr scene_point_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    std::vector<BOX2D> box2d_scene;
    std::vector<BOX3D> box3d_scene;

    Eigen::Matrix4f external_para = Eigen::Matrix4f::Zero();
    Eigen::Matrix3f internal_para = Eigen::Matrix3f::Zero();

    read_data(//image , "../cooperative-vehicle-infrastructure-example_12649705822621696/cooperative-vehicle-infrastructure-example/infrastructure-side/image/000010.jpg",
        scene_point_cloud, "../cooperative-vehicle-infrastructure-example_12649705822621696/cooperative-vehicle-infrastructure-example/infrastructure-side/velodyne/000010.pcd", 
        box2d_scene, "../cooperative-vehicle-infrastructure-example_12649705822621696/cooperative-vehicle-infrastructure-example/infrastructure-side/label/camera/000010.json", 
        box3d_scene, "../cooperative-vehicle-infrastructure-example_12649705822621696/cooperative-vehicle-infrastructure-example/infrastructure-side/label/virtuallidar/000010.json",
        external_para, "../cooperative-vehicle-infrastructure-example_12649705822621696/cooperative-vehicle-infrastructure-example/infrastructure-side/calib/virtuallidar_to_camera/000010.json",
        internal_para, "../cooperative-vehicle-infrastructure-example_12649705822621696/cooperative-vehicle-infrastructure-example/infrastructure-side/calib/camera_intrinsic/000010.json"        
        );
    
    std::cout << "scene_point_cloud size: " << scene_point_cloud->size() << std::endl;
    std::cout << "box2d_scene size: " << box2d_scene.size() << std::endl;
    std::cout << "box3d_scene size: " << box3d_scene.size() << std::endl;
    //cout internal_para
    std::cout << "internal_para: " << internal_para << std::endl;
    //cout external_para
    std::cout << "external_para: " << external_para << std::endl;

    // for (int i = 0; i < box2d_scene.size(); i++) {
    //     std::string name_2d = "box2d_" + std::to_string(i);
    //     viewer.addCube(box2d_scene[i].xmin, box2d_scene[i].xmax, box2d_scene[i].ymin, box2d_scene[i].ymax, 0, 0, 1, 0, 0, name_2d);
    // }

    show_boxed_target_with_color(scene_point_cloud, box3d_scene);
    // 框框和点云目标并不完全重合，看起来这个json的检测框不是针对这个点云的？#todo  

    // calculate_3dboxed_point_cloud_retain2d_rate(scene_point_cloud, box3d_scene, box2d_scene, external_para, internal_para);

    // pcl::PointCloud<pcl::PointXYZI>::Ptr scene_point_cloud_i(new pcl::PointCloud<pcl::PointXYZI>);
    // pcl::copyPointCloud(*scene_point_cloud, *scene_point_cloud_i);
    // for (int i = 0; i < scene_point_cloud_i->size(); i++) {
    //     scene_point_cloud_i->points[i].intensity = 1;
    // }// intensity 的范围 #todo
    // cv::Mat output_image;
    // project2image(scene_point_cloud_i, image, output_image, external_para, internal_para);

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
