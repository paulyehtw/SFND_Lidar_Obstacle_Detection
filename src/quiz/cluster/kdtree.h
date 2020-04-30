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

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		return ids;
	}
};
