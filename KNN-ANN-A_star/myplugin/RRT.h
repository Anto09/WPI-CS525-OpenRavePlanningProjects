#ifndef RRT_H
#define RRT_H

#include <vector>
#include <algorithm>
#include <cmath>
#include <openrave/plugin.h>
#include <boost/bind.hpp>

using namespace OpenRAVE;
using namespace std;

class RRTNode
{
	private:
		vector<float> _config;
		RRTNode* _parent;
		int index = -1; //index in the vector

	public:
		//constructors
		RRTNode() {}
		RRTNode(const vector<float>& config, const RRTNode& p)
		{
			this._config.reserve(config.size())
			copy(this._config, this._config + config.size(), config.begin());

			this._parent = p;
		}

		//Get functions
		Node& GetParent(){ return this._parent; };
		vector<float> GetConfig(){ return this._config; }

		//functions to be defined outside header
		Node& NearestNeighbor();
		double Distance(RRTNode& n);
};

class NodeTree
	private:
		vector<RRTNode> _nodes;
		float _minDist = 0f;
		float _size = 0;
	public:
		//constructors
		NodeTree() {}
		NodeTree(const vector<RRTNode>& nodes)
		{
			this._nodes.reserve(_nodes.size())
			copy(this._nodes, this._nodes + _nodes.size(), _nodes.begin());
		}

		//Get functions
		float GetMinDist(){ return this._minDist; }
		RRTNode& GetNode(int index)
		{ 
			if (index >= this.size)
				return NULL;
			return this._nodes[index];
		}
		vector<RRTNode> GetNodes(){ return this._nodes; }

		//functions to be defined outside header
		void AddNode(const RRTNode& node);
		void DeleteNode(const RRTNode& node);
};

#endif