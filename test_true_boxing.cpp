#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include <pcl/common/common.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/crop_box.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/project_inliers.h>

// #include <opencv2/opencv.hpp>

#include <Eigen/Dense>
#include <Eigen/Core>

#include <jsoncpp/json/json.h>

#include <iostream>
#include <map>
#include <algorithm>

// pcl中筛选离群点的内嵌算法

// 通过 个体点云点留存率 统计 整体点云留存率 的指标

// 根据不同的类别给框上不同的颜色，观察真值的每一类对象本身的提取效果（后期效果不好的一个调整思路


struct BOX2D{
    double xmin, ymin, xmax, ymax;
    BOX2D(double xmin, double ymin, double xmax, double ymax) : xmin(xmin), ymin(ymin), xmax(xmax), ymax(ymax) {}
    BOX2D() : xmin(0), ymin(0), xmax(0), ymax(0) {}
};
struct BOX3D{
    std::string type;
    double yaw;
    struct {
        double h, w, l;
    } dimensions;
    struct {
        double x, y, z;
    } location;

    BOX3D(std::string type, double yaw, double h, double w, double l, double x, double y, double z) : type(type), yaw(yaw), dimensions{h, w, l}, location{x, y, z} {}
    BOX3D() : type(""), yaw(0), dimensions{0, 0, 0}, location{0, 0, 0} {}
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

pcl::PointCloud<pcl::PointXYZ>::Ptr infra_point_cloud(new pcl::PointCloud<pcl::PointXYZ>), vehicle_point_cloud(new pcl::PointCloud<pcl::PointXYZ>);
std::vector<BOX2D> box2d_scene;
std::vector<BOX3D> cooperative_3dbox_scene, infra_3dbox_scene, vehicle_3dbox_scene;

Eigen::Matrix4f T_vehicle_lidar2novatel = Eigen::Matrix4f::Zero();
Eigen::Matrix4f T_vehicle_novatel2world = Eigen::Matrix4f::Zero();
Eigen::Matrix4f T_infra_lidar2world = Eigen::Matrix4f::Zero();
Eigen::Matrix3f internal_para = Eigen::Matrix3f::Zero();


bool read_point_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr result_point_cloud, std::string file_name){

    if(pcl::io::loadPCDFile<pcl::PointXYZ>(file_name, * result_point_cloud) == -1)
    {    
        PCL_ERROR("Couldn't read file test_pcd.pcd \n");
        return  false;
    }
    std::cout << "read point cloud success" << endl;
    return true;

}

bool parseJsonFile(const std::string& file_name, const std::function<void(const Json::Value&)> read){
    std::ifstream ifs;
    ifs.open(file_name); 
    if(!ifs.is_open()) {
        std::cout << "fail to open : " << file_name << std::endl;
        assert(ifs.is_open());
    }
    
    Json::Value json_tree;
    Json::Reader reader;
    if (!reader.parse(ifs, json_tree, false))
    {
        cerr << "parse " + file_name + "failed \n";
        return false;
    }    

    read(json_tree);
    ifs.close();
    return true;
}

void show_3dbox_detailed_type(std::vector<BOX3D> box3d_scene, std::string shown_name){
    std::map<std::string, int> type_count;
    for(auto box3d: box3d_scene){
        type_count[box3d.type]++;
    }
    std::cout << shown_name << ":" << box3d_scene.size() << std::endl;
    for(auto pair: type_count){
        std::cout << "\t" << pair.first << " " << pair.second << std::endl;
    }

}


// void show_boxed_target_with_color(pcl::PointCloud<pcl::PointXYZ>::Ptr scene_point_cloud, std::vector<BOX3D> box3d_scene){

//     show_boxed_target_with_color(scene_point_cloud, box3d_scene, Eigen::Vector4f())
// }

void show_boxed_target_with_color(pcl::PointCloud<pcl::PointXYZ>::Ptr scene_point_cloud, std::vector<BOX3D> box3d_scene, Eigen::Matrix4f T, std::string point_cloud_scene_name){

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

    pcl::visualization::PCLVisualizer viewer(point_cloud_scene_name);
    viewer.setBackgroundColor(0.0, 0.0, 0.0);
    viewer.addCoordinateSystem(1.0);

    for (int i = 0; i < box3d_scene.size(); i++) { 

        pcl::CropBox<pcl::PointXYZRGB> crop_box_filter;
        crop_box_filter.setInputCloud(scene_point_cloud_rgb);
        crop_box_filter.setMin(Eigen::Vector4f(-box3d_scene[i].dimensions.l / 2, -box3d_scene[i].dimensions.w / 2, -box3d_scene[i].dimensions.h / 2, 1.0));
        crop_box_filter.setMax(Eigen::Vector4f(box3d_scene[i].dimensions.l / 2, box3d_scene[i].dimensions.w / 2, box3d_scene[i].dimensions.h / 2, 1.0));
        crop_box_filter.setNegative(false);//默认false，保留box内的点

        crop_box_filter.setTranslation(Eigen::Vector3f(box3d_scene[i].location.x, box3d_scene[i].location.y, box3d_scene[i].location.z));
        crop_box_filter.setRotation(Eigen::Vector3f(0.0f, 0.0f, (float) (box3d_scene[i].yaw * M_PI / 180)));  
        // crop_box_filter.setTransform(Eigen::Affine3f(T));
        
        crop_box_filter.filter(inliers->indices);
        for(int i = 0; i < inliers->indices.size(); i++){
            auto& point = scene_point_cloud_rgb->at(inliers->indices.at(i));
            point.r = 255;
            point.g = 0;
            point.b = 0;
        }

        

        Eigen::Vector4f cubeMax = crop_box_filter.getMax(), cubeMin = crop_box_filter.getMin();
        int x_length = cubeMax[0] - cubeMin[0], y_length = cubeMax[1] - cubeMin[1], z_length = cubeMax[2] - cubeMin[2];
        Eigen::Vector3f cube_translation = crop_box_filter.getTranslation(), cube_rotation = crop_box_filter.getRotation();
        // const Eigen::Quaternionf rotation_quanter(Eigen::Affine3f(cube_rotation));
        // Eigen::Quaternionf rotation_quanter;
        // rotation_quanter = cube_rotation;

        // boxed target
        // viewer.addCube(cubeMin[0], cubeMax[0], cubeMin[1], cubeMax[1], cubeMin[2], cubeMax[2], 1, 0, 0, "box3d_" + std::to_string(i));
        // viewer.addCube(cube_translation, rotation_quanter, x_length, y_length, z_length, "box3d_" + std::to_string(i), 0);



        // viewer.addCube()

        


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

    viewer.addPointCloud<pcl::PointXYZRGB> (scene_point_cloud_rgb, point_cloud_scene_name);
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, point_cloud_scene_name);
   
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

//多窗口点云显示函数
boost::shared_ptr<pcl::visualization::PCLVisualizer> Show2ViewerWindow(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud01, std::string cloud1_name,
                                                                        pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud02, std::string cloud2_name)
{
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->initCameraParameters();
	//创建视窗的标准代码
 
	//第一个窗口显示内容进行设定
	int v1(0);
	viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
	viewer->setBackgroundColor(0, 0, 0, v1);
	viewer->addText(cloud1_name, 10, 10, "v1 text", v1);
	pcl::visualization::PointCloudColorHandlerCustom < pcl::PointXYZ>	rgb(cloud01, 255, 0, 0);
	//pcl::visualization::PointCloudColorHandlerRGBField < pcl::PointXYZRGB>	rgb(pointCloudPtr);
	viewer->addPointCloud<pcl::PointXYZ>(cloud01, rgb, "sample cloud1", v1);
 
	//第二个显示内容进行设定
	int v2(1);
	viewer->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
	viewer->setBackgroundColor(0.3, 0.3, 0.3, v2);
	viewer->addText(cloud2_name, 10, 10, "v2 text", v2);
	pcl::visualization::PointCloudColorHandlerCustom < pcl::PointXYZ>	single_color(cloud02, 0, 255, 0);
	viewer->addPointCloud<pcl::PointXYZ>(cloud02, single_color, "sample cloud2", v2);
 
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud1");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud2");
	viewer->addCoordinateSystem(1.0);
	return viewer;
}

void show_2dlidarimage_project2ground(pcl::PointCloud<pcl::PointXYZ>::Ptr scene_point_cloud, std::vector<BOX3D> box3d_scene, std::string point_cloud_scene_name){
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr boxed_target_point_cloud_set(new pcl::PointCloud<pcl::PointXYZ>());

    // pcl::visualization::PCLVisualizer viewer(point_cloud_scene_name);
    // viewer.setBackgroundColor(0.0, 0.0, 0.0);
    // viewer.addCoordinateSystem(1.0);
    
    int cnt = 0;

    for (int i = 0; i < box3d_scene.size(); i++) { 

        pcl::CropBox<pcl::PointXYZ> crop_box_filter;
        crop_box_filter.setInputCloud(scene_point_cloud);
        crop_box_filter.setMin(Eigen::Vector4f(-box3d_scene[i].dimensions.l / 2, -box3d_scene[i].dimensions.w / 2, -box3d_scene[i].dimensions.h / 2, 1.0));
        crop_box_filter.setMax(Eigen::Vector4f(box3d_scene[i].dimensions.l / 2, box3d_scene[i].dimensions.w / 2, box3d_scene[i].dimensions.h / 2, 1.0));
        crop_box_filter.setNegative(false);//默认false，保留box内的点

        crop_box_filter.setTranslation(Eigen::Vector3f(box3d_scene[i].location.x, box3d_scene[i].location.y, box3d_scene[i].location.z));
        crop_box_filter.setRotation(Eigen::Vector3f(0.0f, 0.0f, (float) (box3d_scene[i].yaw * M_PI / 180)));  
        // crop_box_filter.setTransform(Eigen::Affine3f(T));
        
        pcl::PointCloud<pcl::PointXYZ>::Ptr boxed_target_point_cloud(new pcl::PointCloud<pcl::PointXYZ>());
        crop_box_filter.filter(*boxed_target_point_cloud);
        // std::cout << "boxed_target_point_cloud.size()" << boxed_target_point_cloud->size() << std::endl;
        *boxed_target_point_cloud_set += *boxed_target_point_cloud;
        // std::cout << "boxed_target_point_cloud_set.size()" << boxed_target_point_cloud_set->size() << std::endl;
    }

	pcl::PointCloud<pcl::PointXYZ>::Ptr boxed_target_point_cloud_set_projected(new pcl::PointCloud<pcl::PointXYZ>);
 
 
	//创建一个系数为X = Y = 0; Z = 1的平面
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
	coefficients->values.resize(4);
	coefficients->values[0] = coefficients->values[1] = 0;
	coefficients->values[2] = 1.0;
	coefficients->values[3] = 0;
	//创建滤波器对象
 
	pcl::ProjectInliers<pcl::PointXYZ> proj;
	proj.setModelType(pcl::SACMODEL_PLANE);
	proj.setInputCloud(boxed_target_point_cloud_set);
	proj.setModelCoefficients(coefficients);
	proj.filter(*boxed_target_point_cloud_set_projected);
 
 
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
	viewer = Show2ViewerWindow(boxed_target_point_cloud_set, "infrastructure point cloud", boxed_target_point_cloud_set_projected, "infrastructure point cloud bev ");
 
	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		// boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}




    // // viewer.addPointCloud<pcl::PointXYZ> (boxed_target_point_cloud_set, point_cloud_scene_name);
    // viewer.addPointCloud (boxed_target_point_cloud_set, point_cloud_scene_name);
    // viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, point_cloud_scene_name);
   
    // viewer.addCoordinateSystem (1.0);
    // viewer.setCameraPosition(0, 0, 0, )
    // // viewer.initCameraParameters(); // 啥作用？

    // while (!viewer.wasStopped())
    // {
    //     // viewer.spinOnce();
    //     viewer.spinOnce(100);
    //     // std::this_thread::sleep_for(100ms);
    // }

    // return;
}

int main(){
    // cv::Mat image;
    
    // read 模块有待用接口抽象几层 #todo
    
    std::string prefix_folder = "../cooperative-vehicle-infrastructure-example_12649705822621696/cooperative-vehicle-infrastructure-example",
                infra_point_cloud_filename = "/infrastructure-side/velodyne/000049.pcd", 
                vehicle_point_cloud_filename = "/vehicle-side/velodyne/015404.pcd",
                cooperative_3dbox_filename = "/cooperative/label_world/015404.json",
                infra_3dbox_filename = "/infrastructure-side/label/virtuallidar/000049.json",
                vehicle_3dbox_filename = "/vehicle-side/label/lidar/015404.json",
                vehicle_lidar2novatel_filename = "/vehicle-side/calib/lidar_to_novatel/015404.json",
                vehicle_novatel2world_filename = "/vehicle-side/calib/novatel_to_world/015404.json",
                infra_lidar2world_filename = "/infrastructure-side/calib/virtuallidar_to_world/000049.json";

    infra_point_cloud_filename = prefix_folder + infra_point_cloud_filename;
    vehicle_point_cloud_filename = prefix_folder + vehicle_point_cloud_filename;
    cooperative_3dbox_filename = prefix_folder + cooperative_3dbox_filename;
    infra_3dbox_filename = prefix_folder + infra_3dbox_filename;
    vehicle_3dbox_filename = prefix_folder + vehicle_3dbox_filename;
    vehicle_lidar2novatel_filename = prefix_folder + vehicle_lidar2novatel_filename;
    vehicle_novatel2world_filename = prefix_folder + vehicle_novatel2world_filename;
    infra_lidar2world_filename = prefix_folder + infra_lidar2world_filename;
    

    read_point_cloud(infra_point_cloud, infra_point_cloud_filename);
    read_point_cloud(vehicle_point_cloud, vehicle_point_cloud_filename);

    //3dbox
    parseJsonFile(cooperative_3dbox_filename, [&](const Json::Value json_tree_3dbox){
        for (unsigned int i = 0; i < json_tree_3dbox.size(); i++) {
            std::string type = json_tree_3dbox[i]["type"].asString();
            std::transform(type.begin(), type.end(), type.begin(), ::tolower);
            cooperative_3dbox_scene.push_back(BOX3D(type, 0.0, json_tree_3dbox[i]["3d_dimensions"]["h"].asDouble(), json_tree_3dbox[i]["3d_dimensions"]["w"].asDouble(), json_tree_3dbox[i]["3d_dimensions"]["l"].asDouble(), 
                                        json_tree_3dbox[i]["3d_location"]["x"].asDouble(), json_tree_3dbox[i]["3d_location"]["y"].asDouble(), json_tree_3dbox[i]["3d_location"]["z"].asDouble()));
        }
    });
    
    parseJsonFile(infra_3dbox_filename, [&](const Json::Value json_tree_3dbox){
        for (unsigned int i = 0; i < json_tree_3dbox.size(); i++) {
            std::string type = json_tree_3dbox[i]["type"].asString();
            std::transform(type.begin(), type.end(), type.begin(), ::tolower);
            infra_3dbox_scene.push_back(BOX3D(type, json_tree_3dbox[i]["rotation"].asDouble(), json_tree_3dbox[i]["3d_dimensions"]["h"].asDouble(), json_tree_3dbox[i]["3d_dimensions"]["w"].asDouble(), json_tree_3dbox[i]["3d_dimensions"]["l"].asDouble(), 
                                        json_tree_3dbox[i]["3d_location"]["x"].asDouble(), json_tree_3dbox[i]["3d_location"]["y"].asDouble(), json_tree_3dbox[i]["3d_location"]["z"].asDouble()));
        }
    });

    parseJsonFile(vehicle_3dbox_filename, [&](const Json::Value json_tree_3dbox){
        for (unsigned int i = 0; i < json_tree_3dbox.size(); i++) {
            std::string type = json_tree_3dbox[i]["type"].asString();
            std::transform(type.begin(), type.end(), type.begin(), ::tolower);
            vehicle_3dbox_scene.push_back(BOX3D(type, json_tree_3dbox[i]["rotation"].asDouble(), json_tree_3dbox[i]["3d_dimensions"]["h"].asDouble(), json_tree_3dbox[i]["3d_dimensions"]["w"].asDouble(), json_tree_3dbox[i]["3d_dimensions"]["l"].asDouble(), 
                                        json_tree_3dbox[i]["3d_location"]["x"].asDouble(), json_tree_3dbox[i]["3d_location"]["y"].asDouble(), json_tree_3dbox[i]["3d_location"]["z"].asDouble()));
        }
    });

    // external
    parseJsonFile(vehicle_lidar2novatel_filename, [&](const Json::Value json_tree_external){
        for(unsigned int i = 0; i < 3; i++){
            for(unsigned int j = 0; j < 3; j++){
                T_vehicle_lidar2novatel(i, j) = json_tree_external["transform"]["rotation"][i][j].asDouble();
            }
            T_vehicle_lidar2novatel(i, 3) = json_tree_external["transform"]["translation"][i][0].asDouble();
        }
        T_vehicle_lidar2novatel(3, 3) = 1;
    });

    parseJsonFile(vehicle_novatel2world_filename, [&](const Json::Value json_tree_external){
        for(unsigned int i = 0; i < 3; i++){
            for(unsigned int j = 0; j < 3; j++){
                T_vehicle_novatel2world(i, j) = json_tree_external["rotation"][i][j].asDouble();
            }
            T_vehicle_novatel2world(i, 3) = json_tree_external["translation"][i][0].asDouble();
        }
        T_vehicle_novatel2world(3, 3) = 1;
    });

    parseJsonFile(infra_lidar2world_filename, [&](const Json::Value json_tree_external){
        for(unsigned int i = 0; i < 3; i++){
            for(unsigned int j = 0; j < 3; j++){
                T_infra_lidar2world(i, j) = json_tree_external["rotation"][i][j].asDouble();
            }
            T_infra_lidar2world(i, 3) = json_tree_external["translation"][i][0].asDouble();
        }
        T_infra_lidar2world(3, 3) = 1;
    });

    // internal
    // result = result && parseJsonFile(internal_para_file_name, [&internal_para](const Json::Value json_tree_inter){
    //     for(unsigned int i = 0; i < json_tree_inter["cam_K"].size(); i++){
    //         internal_para(i / 3, i % 3) = json_tree_inter["cam_K"][i].asDouble();
    //     }
    // });

    std::cout << "infra_point_cloud size: " << infra_point_cloud->size() << std::endl;
    std::cout << "vehicle_point_cloud size: " << vehicle_point_cloud->size() << std::endl;    
    // std::cout << "cooperative_3dbox_scene size: " << cooperative_3dbox_scene.size() << std::endl;
    // std::cout << "infra_3dbox_scene size: " << infra_3dbox_scene.size() << std::endl;
    // std::cout << "vehicle_3dbox_scene size: " << vehicle_3dbox_scene.size() << std::endl;
    std::cout << "T_vehicle_lidar2novatel:\n" << T_vehicle_lidar2novatel << std::endl;
    std::cout << "T_vehicle_novatel2world:\n" << T_vehicle_novatel2world << std::endl;
    std::cout << "T_infra_lidar2world:\n" << T_infra_lidar2world << std::endl;


    show_3dbox_detailed_type(cooperative_3dbox_scene, "cooperative");
    show_3dbox_detailed_type(infra_3dbox_scene, "infrastructure");
    show_3dbox_detailed_type(vehicle_3dbox_scene, "vehicle");
    

    // for (int i = 0; i < box2d_scene.size(); i++) {
    //     std::string name_2d = "box2d_" + std::to_string(i);
    //     viewer.addCube(box2d_scene[i].xmin, box2d_scene[i].xmax, box2d_scene[i].ymin, box2d_scene[i].ymax, 0, 0, 1, 0, 0, name_2d);
    // }

    // show_boxed_target_with_color(infra_point_cloud, infra_3dbox_scene, T_infra_lidar2world.transpose() * T_vehicle_novatel2world * T_vehicle_lidar2novatel, "infrastructure point cloud");  
    // show_boxed_target_with_color(vehicle_point_cloud, vehicle_3dbox_scene, T_infra_lidar2world.transpose() * T_vehicle_novatel2world * T_vehicle_lidar2novatel, "vehicle point cloud");  

    std::cout << "true external matrix:\n" << T_infra_lidar2world.inverse() * T_vehicle_novatel2world * T_vehicle_lidar2novatel << std::endl;

    // pcl::PointCloud<pcl::PointXYZI>::Ptr scene_point_cloud_i(new pcl::PointCloud<pcl::PointXYZI>);
    // pcl::copyPointCloud(*scene_point_cloud, *scene_point_cloud_i);
    // for (int i = 0; i < scene_point_cloud_i->size(); i++) {
    //     scene_point_cloud_i->points[i].intensity = 1;
    // }// intensity 的范围 #todo
    // cv::Mat output_image;
    // project2image(scene_point_cloud_i, image, output_image, external_para, internal_para);

    // show_2dlidarimage_project2ground(infra_point_cloud, infra_3dbox_scene, "infrastructure boxed point cloud");
    // show_2dlidarimage_project2ground(vehicle_point_cloud, vehicle_3dbox_scene, "vehicle boxed point cloud");

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
    
   cout << "done!!!!!!!!!!!!!"<<endl;

    return  0;
}
