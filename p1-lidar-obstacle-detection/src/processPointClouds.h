// PCL lib Functions for processing point clouds 

#ifndef PROCESSPOINTCLOUDS_H_
#define PROCESSPOINTCLOUDS_H_

#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/transforms.h>
#include <iostream> 
#include <string>  
#include <vector>
#include <ctime>
#include <chrono>
#include "render/box.h"
#include <unordered_set>

// Structure to represent node of kd tree
struct Node
{
    std::vector<float> point;
    int id;
    Node* left;
    Node* right;

    Node(std::vector<float> arr, int setId)
    :   point(arr), id(setId), left(NULL), right(NULL)
    {}
};

template <typename PointT>
struct KdTree_euclidean
{
    Node* root;


    KdTree_euclidean()
    : root(NULL)
    {}

    void inserthelper(Node *&node, uint level, PointT point, int id)
    {
        // Identify the axis
        uint index = level % 3;

        if (node == NULL)
        {
        // convert point.data arr to vector
         std::vector<float> v_point(point.data, point.data + 3);
         node = new Node(v_point, id);
        }
        else if (point.data[index] < node->point[index])
        {
        // Go to left
        inserthelper(node->left, level + 1, point, id);
        }
        else
        {
        // Go to right
        inserthelper(node->right, level + 1, point, id);
        }
    }

    void insert_cloud(typename pcl::PointCloud<PointT>::Ptr cloud)
    {
        for (uint index = 0; index < cloud->points.size(); index++)
        {
           inserthelper(root, 0, cloud->points[index], index);
        }

    }

    void helpersearch(Node *&node, uint depth, std::vector<int> *ids, PointT target, float distanceTol)
    {
        uint id = depth % 3;

        if (node != NULL)
        {
            // Check if nodes x,y,z are with in target +/- distanceTol
            if (((node->point[0] < target.data[0] + distanceTol) && (node->point[0] > target.data[0] - distanceTol)) &&
                  ((node->point[1] < target.data[1] + distanceTol) && (node->point[1] > target.data[1] - distanceTol)) &&
                    ((node->point[2] < target.data[2] + distanceTol) && (node->point[2] > target.data[2] - distanceTol)))
            {
                // calculate distance between node and point
                uint dis = sqrt((node->point[0] - target.data[0]) * (node->point[0] - target.data[0]) + 
                                (node->point[1] - target.data[1]) * (node->point[1] - target.data[1]) + 
                                (node->point[2] - target.data[2]) * (node->point[2] - target.data[2]));

                // if distance < distanceTol then add to vector
                if (dis < distanceTol)
                {
                    ids->push_back(node->id);
                }
            }

            if (target.data[id] - distanceTol < node->point[id])
            {
                helpersearch(node->left, depth + 1, ids, target, distanceTol);

            }
            if (target.data[id] + distanceTol > node->point[id])
            {
                helpersearch(node->right, depth + 1, ids, target, distanceTol);
            }

        }
    }

    std::vector<int> search(PointT target, float distanceTol)
    {
        std::vector<int> ids;
        uint depth = 0;
        uint maxdistance = 0;

        helpersearch(root, depth, &ids, target, distanceTol);
        return ids;
    }

};

template<typename PointT>
class ProcessPointClouds {
public:

    // constructor
    ProcessPointClouds();

    // deconstructor
    ~ProcessPointClouds();

    void numPoints(typename pcl::PointCloud<PointT>::Ptr cloud);

    typename pcl::PointCloud<PointT>::Ptr FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> SegmentPlane_RANSAC(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold);

    std::vector<typename pcl::PointCloud<PointT>::Ptr> Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize);

    std::vector<typename pcl::PointCloud<PointT>::Ptr> Clustering_EUCLIDEAN(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize);

    std::vector<std::vector<int>> euclideanCluster(typename pcl::PointCloud<PointT>::Ptr cloud, KdTree_euclidean<PointT> *tree, float distanceTol, int minSize, int maxSize);

    void Proximity(typename pcl::PointCloud<PointT>::Ptr cloud,std::vector<int> &cluster,std::vector<bool> &processed_f,int idx, KdTree_euclidean<PointT> *tree,float distanceTol, int maxSize);

    Box BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster);

    void savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file);

    typename pcl::PointCloud<PointT>::Ptr loadPcd(std::string file);

    std::vector<boost::filesystem::path> streamPcd(std::string dataPath);

};

#endif /* PROCESSPOINTCLOUDS_H_ */
