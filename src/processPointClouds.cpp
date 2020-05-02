// PCL lib Functions for processing point clouds

#include "processPointClouds.h"

//constructor:
template <typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}

//de-constructor:
template <typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}

template <typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    std::cout << cloud->points.size() << std::endl;
}

template <typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud,
                                                                              float filterRes,
                                                                              Eigen::Vector4f minPoint,
                                                                              Eigen::Vector4f maxPoint)
{

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // TODO:: Fill in the function to do voxel grid point reduction and region based filtering

    // Create a VoxelGrid filter object
    typename pcl::PointCloud<PointT>::Ptr filteredCloud(new pcl::PointCloud<PointT>());
    pcl::VoxelGrid<PointT> vg;
    vg.setInputCloud(cloud);
    vg.setLeafSize(filterRes, filterRes, filterRes);
    vg.filter(*filteredCloud);

    // Create a CropBox filter object
    typename pcl::PointCloud<PointT>::Ptr croppedCloud(new pcl::PointCloud<PointT>());
    pcl::CropBox<PointT> cb;
    cb.setInputCloud(filteredCloud);
    cb.setMin(minPoint);
    cb.setMax(maxPoint);
    cb.filter(*croppedCloud);

    // Crop off the points near the roof
    std::vector<int> roofIndices;
    pcl::CropBox<PointT> cbRoof;
    cbRoof.setInputCloud(croppedCloud);
    cbRoof.setMin(Eigen::Vector4f(-1.5F, -1.7F, -1.0F, 1.0F));
    cbRoof.setMax(Eigen::Vector4f(2.6F, 1.7F, -0.4F, 1.0F));
    cbRoof.filter(roofIndices);
    pcl::PointIndices::Ptr roofCloudIndices(new pcl::PointIndices);
    for (int idx : roofIndices)
    {
        roofCloudIndices->indices.push_back(idx);
    }
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(croppedCloud);
    extract.setIndices(roofCloudIndices);
    extract.setNegative(true);
    extract.filter(*croppedCloud);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return croppedCloud;
}

template <typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr>
ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud)
{
    // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane

    typename pcl::PointCloud<PointT>::Ptr obstacleCloud(new pcl::PointCloud<PointT>());
    typename pcl::PointCloud<PointT>::Ptr planeCloud(new pcl::PointCloud<PointT>());

    for (int i : inliers->indices)
    {
        planeCloud->points.push_back(cloud->points[i]);
    }

    // Extract the inliers
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*obstacleCloud);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstacleCloud,
                                                                                                      planeCloud);
    return segResult;
}

template <typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr>
ProcessPointClouds<PointT>::RansacPlane(const typename pcl::PointCloud<PointT>::Ptr cloud,
                                        const int maxIterations,
                                        const float distanceTol)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    pcl::PointIndices::Ptr inlierCloud(new pcl::PointIndices);
    srand(time(NULL));

    for (int iteration = 0; iteration < maxIterations; iteration++)
    {
        std::vector<int> inliersTemp{};

        // Randomly select 3 points
        while (inliersTemp.size() < 3)
        {
            inliersTemp.push_back(rand() % (cloud->points.size()));
        }

        // Pick those 3 points from the cloud
        float x1, y1, z1, x2, y2, z2, x3, y3, z3;
        auto itr = inliersTemp.begin();
        x1 = cloud->points[*itr].x;
        y1 = cloud->points[*itr].y;
        z1 = cloud->points[*itr].z;
        itr++;
        x2 = cloud->points[*itr].x;
        y2 = cloud->points[*itr].y;
        z2 = cloud->points[*itr].z;
        itr++;
        x3 = cloud->points[*itr].x;
        y3 = cloud->points[*itr].y;
        z3 = cloud->points[*itr].z;

        // By calculating normal vector of vectors v1(point1->point2) and v2(point1->point3)
        // plane coefficients can be deducted as below :
        float A = (y2 - y1) * (z3 - z1) - (z2 - z1) * (y3 - y1);
        float B = (z2 - z1) * (x3 - x1) - (x2 - x1) * (z3 - z1);
        float C = (x2 - x1) * (y3 - y1) - (y2 - y1) * (x3 - x1);
        float D = -(A * x1 + B * y1 + C * z1);

        // Measure distance between every point and fitted line
        // If distance is smaller than threshold count it as inlier
        for (int i = 0; i < cloud->points.size(); i++)
        {
            float x = cloud->points[i].x;
            float y = cloud->points[i].y;
            float z = cloud->points[i].z;
            float numerator = abs(A * x + B * y + C * z + D);
            float denominator = sqrt(A * A + B * B + C * C);
            float d = numerator / denominator;
            if (d <= distanceTol)
            {
                inliersTemp.push_back(i);
            }
        }

        if (inliersTemp.size() > inlierCloud->indices.size())
        {
            inlierCloud->indices = inliersTemp;
        }
    }

    auto endTime = std::chrono::steady_clock::now();
    auto ellapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "Ransac took " << ellapsedTime.count() << " millicesonds" << std::endl;

    return SeparateClouds(inlierCloud, cloud);
}

template <typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr>
ProcessPointClouds<PointT>::SegmentPlane(
    typename pcl::PointCloud<PointT>::Ptr cloud,
    int maxIterations,
    float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
    // TODO:: Fill in this function to find inliers for the cloud.

    // Create the segmentation object
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
    pcl::SACSegmentation<PointT> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(maxIterations);
    seg.setDistanceThreshold(distanceThreshold);

    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coefficients);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    return SeparateClouds(inliers, cloud);
}

template <typename PointT>
void ProcessPointClouds<PointT>::Proximity(const int index,
                                           const typename pcl::PointCloud<PointT>::Ptr cloud,
                                           typename pcl::PointCloud<PointT>::Ptr cluster,
                                           std::vector<bool> &processed,
                                           KdTree *tree,
                                           const float clusterTolerance)
{
    processed[index] = true;
    // cluster.push_back(index);
    cluster->points.push_back(cloud->points[index]);

    std::vector<int> nearbyPointIndices = tree->searchNNs(cloud->points[index], clusterTolerance);

    for (int id : nearbyPointIndices)
    {
        if (processed[id] == false)
        {
            Proximity(id, cloud, cluster, processed, tree, clusterTolerance);
        }
    }
}

template <typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr>
ProcessPointClouds<PointT>::euclideanClustering(const typename pcl::PointCloud<PointT>::Ptr cloud,
                                                const float clusterTolerance,
                                                const int minSize,
                                                const int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    KdTree *tree(new KdTree);
    tree->insertCloud(cloud);

    std::vector<bool> processed(cloud->points.size(), false);

    for (size_t idx = 0; idx < cloud->points.size(); ++idx)
    {
        if (processed[idx] == false)
        {
            std::vector<int> cluster_idx;
            typename pcl::PointCloud<PointT>::Ptr cluster(new pcl::PointCloud<PointT>);

            Proximity(idx, cloud, cluster, processed, tree, clusterTolerance);

            if (cluster->points.size() >= minSize && cluster->points.size() <= maxSize)
            {
                clusters.push_back(cluster);
            }
        }
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "euclidean clustering took " << elapsedTime.count()
              << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}

template <typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr>
ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud,
                                       float clusterTolerance,
                                       int minSize,
                                       int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles
    typename pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    tree->setInputCloud(cloud);

    std::vector<pcl::PointIndices> clusterIndicesVector;
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance(clusterTolerance);
    ec.setMinClusterSize(minSize);
    ec.setMaxClusterSize(maxSize);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(clusterIndicesVector);

    for (pcl::PointIndices clusterIndices : clusterIndicesVector)
    {
        typename pcl::PointCloud<PointT>::Ptr cloudPtr(new pcl::PointCloud<PointT>);
        for (int index : clusterIndices.indices)
        {
            cloudPtr->points.push_back(cloud->points[index]);
        }
        clusters.push_back(cloudPtr);
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}

template <typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster)
{

    // Find bounding box for one of the clusters
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cluster, minPoint, maxPoint);

    Box box;
    box.x_min = minPoint.x;
    box.y_min = minPoint.y;
    box.z_min = minPoint.z;
    box.x_max = maxPoint.x;
    box.y_max = maxPoint.y;
    box.z_max = maxPoint.z;

    return box;
}

template <typename PointT>
void ProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file)
{
    pcl::io::savePCDFileASCII(file, *cloud);
    std::cerr << "Saved " << cloud->points.size() << " data points to " + file << std::endl;
}

template <typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file)
{

    typename pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT>(file, *cloud) == -1) //* load the file
    {
        PCL_ERROR("Couldn't read file \n");
    }
    std::cerr << "Loaded " << cloud->points.size() << " data points from " + file << std::endl;

    return cloud;
}

template <typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{

    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;
}