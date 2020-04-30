/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "../../render/render.h"

// Structure to represent node of kd tree
struct Node
{
	std::vector<float> point;
	int id;
	Node *left;
	Node *right;

	Node(std::vector<float> arr, int setId)
		: point(arr), id(setId), left(NULL), right(NULL)
	{
	}
};

struct KdTree
{
	Node *root;

	KdTree()
		: root(NULL)
	{
	}

	unsigned binaryDepth : 1;

	void insertNode(const std::vector<float> point, const int id, Node *nodePtr)
	{
		if (point[binaryDepth] < nodePtr->point[binaryDepth])
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

	void insert(std::vector<float> point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root
		binaryDepth = 0;
		if (root == NULL)
		{
			root = new Node(point, id);
		}
		else
		{
			insertNode(point, id, root);
		}
	}

	void checkNode(const std::vector<float> target,
				   Node *node,
				   const float distanceTol,
				   std::vector<int> &ids)
	{
		if (node != NULL)
		{
			if ((abs(node->point[0] - target[0]) <= distanceTol) && (abs(node->point[1] - target[1]) <= distanceTol))
			{
				float distance = sqrt((node->point[0] - target[0]) * (node->point[0] - target[0]) +
									  (node->point[1] - target[1]) * (node->point[1] - target[1]));
				if (distance <= distanceTol)
				{
					ids.push_back(node->id);
				}
			}
			if (target[binaryDepth] - distanceTol < node->point[binaryDepth])
			{
				binaryDepth++;
				checkNode(target, node->left, distanceTol, ids);
			}
			if (target[binaryDepth] + distanceTol > node->point[binaryDepth])
			{
				binaryDepth++;
				checkNode(target, node->right, distanceTol, ids);
			}
		}
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
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
