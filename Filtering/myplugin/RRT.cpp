#include <RRT.h>

//RRTNode non-constructor and non-get methods
Node& RRTNode::NearestNode(const NodeTree& tree, kdtree = false)
{
	if (kdtree)
	{

	}
	else
	{
		min_dist = 999999f;
		RRTNode* returnNode = NULL;
		for (int i = 0; i < tree->_nodes.size(); i++)
		{
			Node* curNode = &(tree->_nodes[i]);
			dist = this.distance(curNode);
			if (dist < min_dist)
			{
				min_dist = dist;
				returnNode = curNode;
			}
		}
	}
}


//NodeTree non-constructor and non-get methods
void NodeTree::AddNode(RRTNode node)
{
	this.size++;
	this._nodes.resize(this.size);
	this._nodes[this.size-1] = node;
	node.index = this.size - 1;
}

void NodeTree::DeleteNode(RRTNode node)
{
	this._nodes.resize(remove(this._nodes.begin(), this._nodes.end(), node) - this._nodes.begin());
}
