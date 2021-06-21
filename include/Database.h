
#ifndef Database_H
#define Database_H

#include <iostream>
#include <fstream>
#include <sstream>

#include <string>
#include <functional>
#include <cfloat>
#include <limits>

#include <set>
#include <map>
#include <vector>
#include <list>
#include <unordered_map>
#include <algorithm>
#include <chrono>
#include <math.h>

using namespace std;

const double TIME_STEP = 2.0;
const double SCALE_MAP_SIZE = 0.4;

typedef vector<vector<bool>> MAP;

// Pose
typedef struct Pose_ {
	double x;
	double y;
	double t;
	bool operator==(const Pose_& p)const {
		return (x == p.x) && (y == p.y) && (t == p.t);
	}
}Pose;


double Round_val(double);
Pose RealtoIndex(Pose);
Pose IndextoReal(Pose);
bool EmptyPose(Pose);
typedef vector<Pose> PATH;

namespace std {
	template<>
	struct hash<Pose>{
		size_t operator()(const Pose& p) const	{
			string s = to_string((int)p.x) + "," + to_string((int)p.y) + "," + to_string((int)p.t);
			return hash<string>{}(s);
		}
	};
}

// State
typedef struct State_ {
	string name;
	string state;
}State;

// Task
class Task {
private:
	string name; // known as action name
	string hltname; // known as high-level task name
	double cost;
	vector<State> precondition;
	vector<State> effect;
	Pose initial_pose;
	Pose final_pose;
	PATH path;
	char type;
	bool done;
public:
	Task();
	Task(string, double, vector<State>, vector<State>, char, bool = false, Pose = { -1,-1,0 }, Pose = { -1,-1,0 });
	void SetPose(Pose, Pose);
	void SetName(string);
	void SetType(char);
	void SethltName(string);
	void SetPrecondtion(vector<State>);
	void SetEffect(vector<State>);
	void SetPath(PATH);
	void SetCost(double);
	void DoneTask(void);
	void UnDoneTask(void);
	bool CheckDone(void);
	string GetName(void);
	string GethltName(void);
	double GetCost(void);
	string GetHighLevelTaskName(void);
	Pose GetInitialPose(void);
	Pose GetFinalPose(void);
	PATH GetPath(void);
	char GetType(void);
	vector<State> GetPrecondition(void);
	vector<State> GetEffect(void);
	string PrintDetails(string);	
	bool operator==(const Task& other) const {
		if(this->name != other.name) return false;
		if(this->cost == other.cost) return false;
		if(!(this->initial_pose == other.initial_pose)) return false;
		if(!(this->final_pose == other.final_pose)) return false;
		if(this->type != other.type) return false;
		if(this->done != other.done) return false;
		return true;
	}
};

// High-level Task
class HighLevelTask{
private:
	string		 name;
	vector<vector<Task>> action_sequence;	
	double		 score;
	double		 start_time;
	double		 end_time;
	bool		 done;
public:
	HighLevelTask();
	HighLevelTask(string, vector<vector<Task>>, double = 1);
	void SetName(string);
	void SetScore(double);
	void SetTimeConstraint(double =-1, double=-1);
	void SetActionSeq(vector<vector<Task>>);	
	void DoneTask(void);
	void UnDoneTask(void);
	bool CheckDone(void);
	string GetName(void);
	double GetScore(void);
	double GetStartTime(void);
	double GetEndTime(void);
	vector<vector<Task>> GetActionSequence(void);
	void UpdateActionSequence(Task, int,int);
	string PrintDetails(string);
	string PrintTaskDetails(string);
	bool operator==(const HighLevelTask& other) const {
		if(this->name != other.name) return false;
		if(!(this->score == other.score)) return false;
		if(this->start_time != other.start_time) return false;
		if(this->end_time != other.end_time) return false;
		if(this->done != other.done) return false;
		for(int i = 0;i < this->action_sequence.size(); i++){
			for(int j = 0;j < this->action_sequence[i].size(); j++){
				if(!(this->action_sequence[i][j] == other.action_sequence[i][j])) return false;
			}
		}
		return true;
	}
};

// Robot
class Robot{
private:
	string		name;
	double		velocity;
	Pose		pose;
	set<char> 	capabilities;
	map<string, string> state;
	double		score;
	double		cost;
	Task		current_task;
	vector<Task> 	next_task;
	vector<Task> 	done_task;
public:
	Robot();
	Robot(string, set<char>, Pose, map<string, string>, double score = 0, double cost = 0, double vel = 1);
	void SetTasks(Task, vector<Task>);
	void SetName(string);
	void SetPose(Pose);
	void SetCapabilities(set<char>);
	void SetState(map<string, string>);
	void UpdateState(State, string = "");
	void SetScore(double);
	void SetCost(double);	
	void SetCurrentTask(Task);
	void SetNextTasks(vector<Task>);
	void AddNextTasks(Task);
	void GetTasks(Task&, vector<Task>&);
	Task GetCurrentTask(void);
	vector<Task> GetNextTasks(void);
	string GetName(void);
	double GetScore(void);
	double GetCost(void);
	Pose GetPose(void);
	set<char> GetCapabilities(void);
	map<string, string> GetState(void);
	string GetCapabilitiesString(void);
	string GetStateString(void);
	string PrintDetails(string);
	bool operator==(const Robot& other) const {
		if(this->name != other.name) return false;
		if(!(this->pose == other.pose)) return false;
		if(this->score != other.score) return false;
		if(this->capabilities != other.capabilities) return false;
		if(this->state != other.state) return false;
		return true;
	}
};

// world state
class WorldState{
private:
	vector<Robot>	robots;
	vector<HighLevelTask>	tasks;
public:
	WorldState();
	WorldState(vector<Robot>&, vector<HighLevelTask>&);
	void SetRobot(vector<Robot>);
	void SetTasks(vector<HighLevelTask> );
	vector<Robot>			GetRobot(void)const;
	vector<HighLevelTask>	GetTasks(void)const;
	vector<double> CalculateCost(void);
	void UpdateTask(vector<HighLevelTask>);
	bool CheckAvailableTask(void);
	string PrintDetails(string);

	bool operator==(const WorldState& other) const {
		for(int i = 0; i < this->robots.size(); i++) {
			if(!(this->robots[i] == other.robots[i])) return false;
		}
		for(int i = 0; i < this->tasks.size(); i++) {
			if(!(this->tasks[i] == other.tasks[i])) return false;
		}
		return true;
	}
};
namespace std {
	template <>
	struct hash<WorldState>{
		size_t operator()(const WorldState& ws) const{
			string s = "";
			for (auto r : ws.GetRobot()) {
				s += r.GetName();
				s += r.GetStateString();
				s += to_string((int)r.GetPose().x) + "," + to_string((int)r.GetPose().y);
				s += to_string(r.GetScore());
			}
			for (auto t : ws.GetTasks()) {
				s += t.GetName() + ",";
				if(t.CheckDone()) s+= "1";
				else s+="0";
			}
			return hash<string>{}(s);
		}
	};
}

class Database{
private:
	WorldState world_state;
	MAP map;
public:
	Database();
	Database(WorldState, MAP);
	MAP GetMap(void);
	WorldState GetCurrentState(void);
	void SetCurrentState(WorldState);
	bool CheckAvailableTask(void);
};

#endif

