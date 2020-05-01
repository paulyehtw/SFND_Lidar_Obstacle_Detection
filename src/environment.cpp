/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "processPointClouds.h"
#include "render/render.h"
#include "sensors/lidar.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"

std::vector<Car> initHighway(bool renderScene, pcl::visualization::PCLVisualizer::Ptr &viewer)
{

    Car egoCar(Vect3(0, 0, 0), Vect3(4, 2, 2), Color(0, 1, 0), "egoCar");
    Car car1(Vect3(15, 0, 0), Vect3(4, 2, 2), Color(0, 0, 1), "car1");
    Car car2(Vect3(8, -4, 0), Vect3(4, 2, 2), Color(0, 0, 1), "car2");
    Car car3(Vect3(-12, 4, 0), Vect3(4, 2, 2), Color(0, 0, 1), "car3");

    std::vector<Car> cars;
    cars.push_back(egoCar);
    cars.push_back(car1);
    cars.push_back(car2);
    cars.push_back(car3);

    if (renderScene)
    {
        renderHighway(viewer);
        egoCar.render(viewer);
        car1.render(viewer);
        car2.render(viewer);
        car3.render(viewer);
    }

    return cars;
}

void cityBlock(pcl::visualization::PCLVisualizer::Ptr &viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display City Block     -----
    // ----------------------------------------------------

    ProcessPointClouds<pcl::PointXYZI> *pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
    // Load the raw cloud data
    pcl::PointCloud<pcl::PointXYZI>::Ptr rawCloud =
        pointProcessorI->loadPcd("../src/sensors/data/pcd/data_1/0000000000.pcd");

    // Preprocess the raw cloud - filtering and cropping
    pcl::PointCloud<pcl::PointXYZI>::Ptr filteredCloud =
        pointProcessorI->FilterCloud(rawCloud,
                                     0.2F,
                                     Eigen::Vector4f(-10.0F, -10.0F, -5.0F, 1.0F),
                                     Eigen::Vector4f(30.0F, 10.0F, 5.0F, 1.0F));
    // Segment the filtered cloud into obstacle cloud and plane cloud
    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentedCloud =
        pointProcessorI->SegmentPlane(filteredCloud, 100, 0.2);

    // Cluster obstacles in obstacleCloud
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> clusters =
        pointProcessorI->Clustering(segmentedCloud.first, 0.5F, 10, 700);
    std::vector<Color> clusterColors{Color(1, 0, 0), Color(1, 1, 0), Color(0, 0, 1)};
    size_t clusterId = 0;
    for (auto cluster : clusters)
    {
        Box box = pointProcessorI->BoundingBox(cluster);
        renderBox(viewer, box, clusterId);
        renderPointCloud(viewer, cluster,
                         "obstacleCloud" + std::to_string(clusterId),
                         clusterColors[clusterId % clusterColors.size()]);
        clusterId++;
    }
    renderPointCloud(viewer, segmentedCloud.second, "planeCloud", Color(1, 1, 1));
}

void simpleHighway(pcl::visualization::PCLVisualizer::Ptr &viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------

    // RENDER OPTIONS
    bool renderScene = false;
    std::vector<Car> cars = initHighway(renderScene, viewer);

    // TODO:: Create lidar sensor
    Lidar *lidar = new Lidar(cars, 0.0F);
    pcl::PointCloud<pcl::PointXYZ>::Ptr rawCloud = lidar->scan();
    // Color color(1.0F, 1.0F, 1.0F);
    // renderPointCloud(viewer, rawCloud, "Test", color);

    // TODO:: Create point processor
    ProcessPointClouds<pcl::PointXYZ> *pointProcessor = new ProcessPointClouds<pcl::PointXYZ>();
    std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentCloud =
        pointProcessor->SegmentPlane(rawCloud, 100, 0.2);
    // renderPointCloud(viewer, segmentCloud.first, "obstacleCloud", Color(1, 0, 0));
    renderPointCloud(viewer, segmentCloud.second, "planeCloud", Color(1, 1, 1));

    // Cluster obstacles from segmented cloud
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters = pointProcessor->Clustering(segmentCloud.first, 1.0F, 3, 30);
    std::vector<Color> clusterColors{Color(1, 0, 0), Color(1, 1, 0), Color(0, 0, 1)};
    size_t clusterId = 0;
    for (auto cluster : clusters)
    {
        Box box = pointProcessor->BoundingBox(cluster);
        renderBox(viewer, box, clusterId);
        renderPointCloud(viewer, cluster, "obstacleCloud" + std::to_string(clusterId), clusterColors[clusterId]);
        clusterId++;
    }
}

//setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr &viewer)
{

    viewer->setBackgroundColor(0, 0, 0);

    // set camera position and angle
    viewer->initCameraParameters();
    // distance away in meters
    int distance = 16;

    switch (setAngle)
    {
    case XY:
        viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0);
        break;
    case TopDown:
        viewer->setCameraPosition(0, 0, distance, 1, 0, 1);
        break;
    case Side:
        viewer->setCameraPosition(0, -distance, 0, 0, 0, 1);
        break;
    case FPS:
        viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
    }

    if (setAngle != FPS)
        viewer->addCoordinateSystem(1.0);
}

int main(int argc, char **argv)
{
    std::cout << "starting enviroment" << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    CameraAngle setAngle = XY;
    initCamera(setAngle, viewer);
    cityBlock(viewer);

    while (!viewer->wasStopped())
    {
        viewer->spinOnce();
    }
}