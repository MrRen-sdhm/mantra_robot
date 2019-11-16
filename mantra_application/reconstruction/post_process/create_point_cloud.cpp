#include <vector>
#include "opencv2/opencv.hpp"
#include "opencv2/imgproc.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/filter.h>

//#define SHOW

// Load an M x N matrix from a text file (numbers delimited by spaces/tabs)
// Return the matrix as a float vector of the matrix in row-major order
std::vector<float> LoadMatrixFromFile(std::string filename, int M, int N) {
    std::vector<float> matrix;
    FILE *fp = fopen(filename.c_str(), "r");
    for (int i = 0; i < M * N; i++) {
        float tmp;
        int iret = fscanf(fp, "%f", &tmp);
        matrix.push_back(tmp);
    }
    fclose(fp);
    return matrix;
}


void createLookup(cv::Mat &lookupX, cv::Mat &lookupY, size_t width, size_t height, std::vector<float> cameraMatrixVec)
{
    cv::Mat cameraMatrixColor;
    cameraMatrixColor = cv::Mat::zeros(3, 3, CV_64F);

    auto *itC = cameraMatrixColor.ptr<double>(0, 0);
    for(size_t i = 0; i < 9; ++i, ++itC) {
        *itC = cameraMatrixVec[i];
    }

    // std::cout << cameraMatrixColor << std::endl;

    const float fx = 1.0f / cameraMatrixColor.at<double>(0, 0);
    const float fy = 1.0f / cameraMatrixColor.at<double>(1, 1);
    const float cx = cameraMatrixColor.at<double>(0, 2);
    const float cy = cameraMatrixColor.at<double>(1, 2);
    float *it;

    lookupY = cv::Mat(1, height, CV_32F);
    it = lookupY.ptr<float>();
    for(size_t r = 0; r < height; ++r, ++it)
    {
        *it = (r - cy) * fy;
    }

    lookupX = cv::Mat(1, width, CV_32F);
    it = lookupX.ptr<float>();
    for(size_t c = 0; c < width; ++c, ++it)
    {
        *it = (c - cx) * fx;
    }
}

// 创建点云
void create_point_cloud(const cv::Mat &color ,const cv::Mat &depth, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud,
                                cv::Mat lookupX, cv::Mat lookupY) {
    const float badPoint = std::numeric_limits<float>::quiet_NaN();

#pragma omp parallel for
    for(int r = 0; r < depth.rows; ++r)
    {
        pcl::PointXYZRGBA *itP = &cloud->points[r * depth.cols];
        const auto *itD = depth.ptr<uint16_t>(r);
        const auto *itC = color.ptr<cv::Vec3b>(r);
        const float y = lookupY.at<float>(0, r);
        const float *itX = lookupX.ptr<float>();

        for(size_t c = 0; c < (size_t)depth.cols; ++c, ++itP, ++itD, ++itC, ++itX)
        {
            const float depthValue = *itD / 1000.0f;
            // Check for invalid measurements
            if(*itD == 0 || *itD > 1000) {
                // not valid
                itP->x = itP->y = itP->z = badPoint;
                itP->rgba = 0;
                continue;
            }
            itP->z = depthValue;
            itP->x = *itX * depthValue;
            itP->y = y * depthValue;
            itP->b = itC->val[0];
            itP->g = itC->val[1];
            itP->r = itC->val[2];
            itP->a = 255;
        }
    }
}


int main(int argc, char** argv)
{
    std::string color_path;
    std::string depth_path;
    std::string cam_K_dir;
    std::string cam_pose_file;
    std::string save_path;
    std::vector<float> workspace = {0.4, 2.0, -0.4, 0.4, -0.01, 1.0};

    if (argc > 5) {
        color_path = argv[1];
        depth_path = argv[2];
        cam_K_dir = argv[3];
        cam_pose_file = argv[4];
        save_path = argv[5];
    } else {
        printf("[ERROR] Usage: exe_path color_path depth_path cam_K_dir cam_pose_dir save_path\n");
        return -1;
    }

//    std::cout << "cam_K_dir: " << cam_K_dir << std::endl;
//    std::cout << "cam_pose_file: " << cam_pose_file << std::endl;
//    std::cout << "save_path: " << save_path << std::endl;

    if (argc > 11) {
        workspace[0] = atof(argv[6]);
        workspace[1] = atof(argv[7]);
        workspace[2] = atof(argv[8]);
        workspace[3] = atof(argv[9]);
        workspace[4] = atof(argv[10]);
        workspace[5] = atof(argv[11]);
    }

    std::vector<float> cam_K_vec = LoadMatrixFromFile(cam_K_dir + "/camera-intrinsics.txt", 3, 3);
    std::vector<float> cam2world_vec = LoadMatrixFromFile(cam_pose_file, 4, 4);
    Eigen::Matrix4f cam2world_mat;
    cam2world_mat << cam2world_vec[0],  cam2world_vec[1],  cam2world_vec[2],  cam2world_vec[3],
                     cam2world_vec[4],  cam2world_vec[5],  cam2world_vec[6],  cam2world_vec[7],
                     cam2world_vec[8],  cam2world_vec[9],  cam2world_vec[10], cam2world_vec[11],
                     cam2world_vec[12], cam2world_vec[13], cam2world_vec[14], cam2world_vec[15];

    /// 创建点云
    cv::Mat lookupX, lookupY;
    cv::Mat color, depth;
    pcl::PCDWriter writer;

    color = cv::imread(color_path);
    depth = cv::imread(depth_path, -1);

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBA>); // PCL格式点云
    cloud->height = color.rows;
    cloud->width = color.cols;
    cloud->is_dense = false;
    cloud->points.resize(cloud->height * cloud->width);

    createLookup(lookupX, lookupY, color.cols, color.rows, cam_K_vec);
    create_point_cloud(color, depth, cloud, lookupX, lookupY);

    // transform to world coordinate
    pcl::transformPointCloud (*cloud, *cloud, cam2world_mat);

    // 滤除工作空间外点云
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_workspace_filter(new pcl::PointCloud<pcl::PointXYZRGBA>);
    for (int i = 0; i < cloud->size(); i++) {
        const pcl::PointXYZRGBA &p = cloud->points[i];
        if (p.x > workspace[0] && p.x < workspace[1] && p.y > workspace[2] &&
            p.y < workspace[3] && p.z > workspace[4] && p.z < workspace[5]) {
            cloud_workspace_filter->push_back(p);
        }
    }

    // remove nan
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*cloud_workspace_filter, *cloud_workspace_filter, indices);

    // 保存
    writer.writeBinary(save_path, *cloud_workspace_filter);


#ifdef SHOW
    pcl::visualization::PCLVisualizer::Ptr visualizer(new pcl::visualization::PCLVisualizer("Cloud Viewer"));
    const std::string cloudName = "rendered";
    visualizer->addCoordinateSystem(0.2);
    visualizer->addPointCloud(cloud_workspace_filter, cloudName);
    visualizer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, cloudName);
    visualizer->initCameraParameters();
    visualizer->setBackgroundColor(0, 0, 0);
    visualizer->setSize(color.cols, color.rows);
    visualizer->setShowFPS(true);
//    visualizer->setCameraPosition(1, 0, 0, 0, 0, 0);

    while(!visualizer->wasStopped()) {
        visualizer->spinOnce(10);
    }
#endif

    return 0;
}
