#pragma once

#include "Database.h"
#include "AStar.h"

typedef struct Agent_ {
	Robot robot;	
	vector<Task> task;
	vector<pair<Pose,Pose>> target;
	PATH path;
}Agent;

typedef struct Conflict_ {
	Pose conflict_pose;	
	string robot_name;
}Conflict;

class HighLevelNode {
public:
	double cost;
	vector<Conflict> constraint;
	vector<Agent> solution;
	HighLevelNode();
	HighLevelNode(vector<Agent>, vector<Conflict>);

	bool operator<(const HighLevelNode& AnotherNode) const
	{
		return this->cost < AnotherNode.cost;
	}
};

class CBS
{
private:
	AStar astar;
public:
	CBS();
	CBS(AStar);
	vector<Conflict> PreComputeConflict(vector<Agent>);
	vector<Agent> Solve(vector<Agent>, vector<Conflict> ={});
	vector<Conflict> GetConflict(HighLevelNode);
	vector<Agent> ComputeSolution(vector<Agent>,  vector<Conflict> = {});
	double ComputeCost(vector<Agent>);
};

