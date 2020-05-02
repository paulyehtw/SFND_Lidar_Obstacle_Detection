#ifndef KDTREE_H_
#define KDTREE_H_

#include <cmath>
#include <cstdlib>
#include <pcl/common/common.h>
#include <vector>

/// @brief Structure to represent node of kd tree
struct Node
{
	pcl::PointXYZI point;
	int id;
	Node *left;
	Node *right;

	Node(pcl::PointXYZI arr, int setId)
		: point(arr), id(setId), left(NULL), right(NULL)
	{
	}
};

/// @brief KD-Tree datatype
struct KdTree
{
	Node *root;
	unsigned binaryDepth : 1;

	KdTree()
		: root(NULL), binaryDepth(0U)
	{
	}

	void insertNode(const pcl::PointXYZI point, const int id, Node *nodePtr)
	{
		if (binaryDepth == 0U)
		{
			if (point.x < nodePtr->point.x)
			{
				if (nodePtr->left == NULL)
				{
					nodePtr->left = new Node(point, id);
				}
				else
				{
					binaryDepth++;
					insertNode(point, id, nodePtr->left);
				}
			}
			else
			{
				if (nodePtr->right == NULL)
				{
					nodePtr->right = new Node(point, id);
				}
				else
				{
					binaryDepth++;
					insertNode(point, id, nodePtr->right);
				}
			}
		}
		else
		{
			if (point.y < nodePtr->point.y)
			{
				if (nodePtr->left == NULL)
				{
					nodePtr->left = new Node(point, id);
				}
				else
				{
					binaryDepth++;
					insertNode(point, id, nodePtr->left);
				}
			}
			else
			{
				if (nodePtr->right == NULL)
				{
					nodePtr->right = new Node(point, id);
				}
				else
				{
					binaryDepth++;
					insertNode(point, id, nodePtr->right);
				}
			}
		}
	}

	/// @brief  Creates a new node and place correctly with in the root
	void insertCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud)
	{
		binaryDepth = 0;
		for (int id = 0; id < cloud->points.size(); id++)
		{
			if (root == NULL)
			{
				root = new Node(cloud->points[id], id);
			}
			else
			{
				insertNode(cloud->points[id], id, root);
			}
		}
	}

	void checkNode(const pcl::PointXYZI target,
				   Node *node,
				   const float distanceTol,
				   std::vector<int> &ids)
	{
		if (node != NULL)
		{
			if ((abs(node->point.x - target.x) <= distanceTol) &&
				(abs(node->point.y - target.y) <= distanceTol) &&
				(abs(node->point.z - target.z) <= distanceTol))
			{
				float distance = sqrt((node->point.x - target.x) * (node->point.x - target.x) +
									  (node->point.y - target.y) * (node->point.y - target.y) +
									  (node->point.z - target.z) * (node->point.z - target.z));
				if (distance <= distanceTol)
				{
					ids.push_back(node->id);
				}
			}
			if (binaryDepth == 0U)
			{
				if (target.x - distanceTol < node->point.x)
				{
					binaryDepth++;
					checkNode(target, node->left, distanceTol, ids);
				}
				if (target.x + distanceTol > node->point.x)
				{
					binaryDepth++;
					checkNode(target, node->right, distanceTol, ids);
				}
			}
			else
			{
				if (target.y - distanceTol < node->point.y)
				{
					binaryDepth++;
					checkNode(target, node->left, distanceTol, ids);
				}
				if (target.y + distanceTol > node->point.y)
				{
					binaryDepth++;
					checkNode(target, node->right, distanceTol, ids);
				}
			}
		}
	}

	/// @brief Returns a list of nearest neighbor ids in the tree that are within distance of target
	std::vector<int> searchNNs(pcl::PointXYZI target, float distanceTol)
	{
		std::vector<int> ids;
		if (root != NULL)
		{
			binaryDepth = 0;
			checkNode(target, root, distanceTol, ids);
		}
		return ids;
	}
};

#endif /* KDTREE_H_ */