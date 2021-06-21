#pragma once

#include <queue>
#include <unordered_set>
#include <unordered_map>
#include <algorithm>
#include "Database.h"
#include <math.h>

string HASH_POSE(Pose p1, Pose p2);

class AStarNode
{
public:
	Pose state;
	double g;
	double h;
	double f;
	AStarNode* parent;

	AStarNode();
	AStarNode(Pose);
	AStarNode(const AStarNode& other);

	bool operator==(const AStarNode& AnotherNode) const
	{
		return (this->state.x == AnotherNode.state.x) && (this->state.y == AnotherNode.state.y) && (this->state.t == AnotherNode.state.t);
	}

	bool operator<(const AStarNode& AnotherNode) const
	{
		return this->f < AnotherNode.f;
	}
};

struct AStarcomparator {
	bool operator()(const AStarNode& l, const AStarNode& r) {
		return l.f > r.f;
	}
};

namespace std{
template <>
struct hash<AStarNode>
{
	size_t operator()(const AStarNode& n) const
	{
		return hash<Pose>{}(n.state);
	}
};
}

class AStar
{
private:
	MAP map;
	double Heuristic(AStarNode, AStarNode);
	bool CheckClosedSet(AStarNode, unordered_set<AStarNode>);
	vector<Pose> GetNewState(Pose);
	bool CheckGoal(AStarNode, AStarNode);
	bool CheckConflict(AStarNode, PATH); 
	vector<Pose> ReconstructPlan(AStarNode);
public:
	AStar();
	AStar(MAP);
	vector<Pose> Solve(Pose, Pose, PATH = {});
	unordered_map<string, int> GetPathData(WorldState);
};