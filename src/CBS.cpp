#include "CBS.h"

HighLevelNode::HighLevelNode() {}
HighLevelNode::HighLevelNode(vector<Agent> soln, vector<Conflict> constraint) {
	this->solution = soln;
	this->constraint = constraint;
}

CBS::CBS(){}
CBS::CBS(AStar astar){
	this->astar = astar;
}

vector<Conflict> CBS::PreComputeConflict(vector<Agent> agents){
	vector<Conflict> c_vec;
	for(auto& a:agents){
		a.path.clear();
		a.path = a.robot.GetCurrentTask().GetPath();
	}
	for(auto& r1:agents){
		vector<Conflict> conf_vec;
		for(auto& p:r1.robot.GetCurrentTask().GetPath()){
			Conflict c;
			c.conflict_pose = p;
			conf_vec.push_back(c);
		}
		for(auto& r2:agents){
			string r2_name = r2.robot.GetName();
			if(r1.robot.GetName() == r2_name) continue;
			for(auto& c:conf_vec){
				c.robot_name = r2_name;
			}
			c_vec.insert(c_vec.end(), conf_vec.begin(), conf_vec.end());
		}
	}
	return c_vec;
}

vector<Agent> CBS::Solve(vector<Agent> initial_condition, vector<Conflict> conflict_path){
	HighLevelNode start;
	start.constraint = conflict_path;
	start.solution = this->ComputeSolution(initial_condition, conflict_path);
	start.cost = this->ComputeCost(start.solution);
	set<HighLevelNode> open_set;
	open_set.insert(start);

	while (!open_set.empty()){
		HighLevelNode current = *open_set.begin();
		open_set.erase(open_set.begin());
		vector<Conflict> conflict = GetConflict(current);
		if (conflict.size() == 0) {	return current.solution;}
		for (auto& conf:conflict) {
			HighLevelNode new_node = current;
			new_node.constraint.push_back(conf);
			new_node.solution = this->ComputeSolution(new_node.solution, new_node.constraint);
			new_node.cost = this->ComputeCost(new_node.solution);
			open_set.insert(new_node);
		}
	}
	cout << "error on solving CBS !!\n";
	throw std::invalid_argument("cannot find path, all path conflict !!! ");
}

vector<Conflict> CheckVertexConflict(Agent r1,Agent r2){
	vector<Conflict> output;
	int r1t = 0;int r2t = 0;
	while(r1t<r1.path.size() && r2t<r2.path.size()){
		int time_diff = r1.path[r1t].t - r2.path[r2t].t;
		if (time_diff > TIME_STEP){
			r2t++;
		}else if(time_diff < TIME_STEP){
			r1t++;
		}else{
			// time different <= TIme step, that is within 1 step
			// vertex conflict (same time same place)
			// following conflict(diff time same place)
			// same place at all time region:
			if( (r1.path[r1t].x == r2.path[r2t].x) && \
				(r1.path[r1t].y == r2.path[r2t].y)&& \
				 r1.path[r1t].t != 0 &&\
				 r2.path[r2t].t != 0){
				Conflict c1,c2;
				c1.robot_name = r1.robot.GetName();
				c2.robot_name = r2.robot.GetName();
				c1.conflict_pose = r1.path[r1t];
				c2.conflict_pose = r2.path[r2t];
				output.push_back(c1);
				output.push_back(c2);
				Conflict c11 = c1;
				c11.conflict_pose.t += TIME_STEP;
				output.push_back(c11);
				c11 = c1;
				c11.conflict_pose.t -= TIME_STEP;
				output.push_back(c11);
				return output;
			}
			r1t++;
			r2t++;
		}
	}
	//no conflict
	return output;
}

vector<Conflict> CBS::GetConflict(HighLevelNode node) {
	vector<Conflict> output;
	for(int r1=0;r1<node.solution.size();r1++){
		for(int r2=r1+1;r2<node.solution.size();r2++){
			if(node.solution[r1].robot.GetName() == node.solution[r2].robot.GetName() || node.solution[r2].path.size() < 2 || node.solution[r1].path.size() < 2) continue;
			output = CheckVertexConflict(node.solution[r1], node.solution[r2]);
		}
	}
	return output;
}

vector<Agent> CBS::ComputeSolution(vector<Agent> agents_in, vector<Conflict> conflict) {
	// cout << "start compute soln\n";
	vector<Agent> agents = agents_in;
	double max_time = 0;
	for (auto& agent:agents) {
		// conflict path
		PATH conflict_path;
		for (auto& c : conflict) {
			if(c.robot_name == agent.robot.GetName()) conflict_path.push_back(c.conflict_pose);
		}
		// single search
		agent.path.clear();
		for(int i = 0; i < agent.target.size(); i++){
			// cout << agent.robot.GetName() << ":"<< agent.target[i].first.x << "," << agent.target[i].first.y << ":" << agent.target[i].second.x << "," << agent.target[i].second.y<<"\n";
			PATH search_path = this->astar.Solve(agent.target[i].first, agent.target[i].second, conflict_path);
			search_path.insert(search_path.begin(), agent.target[i].first);
			agent.task[i].SetPath(search_path);
			for(int j = 0; j < int(agent.task[i].GetCost()); j++){
				Pose p =search_path.back();
				p.t += 1;
				search_path.push_back(p);
			}
			agent.path.insert(agent.path.end(),search_path.begin(),search_path.end());
			if(i < agent.target.size() - 1){
				agent.target[i+1].first.t = agent.path.back().t;
			} 
		}
		if(agent.path.size() == 0) agent.path.push_back(RealtoIndex(agent.robot.GetPose()));
		if(agent.path.back().t > max_time) max_time = agent.path.back().t;
	}
	for(auto& agent:agents) {
		if(agent.task.size() != 0){
			agent.robot.SetNextTasks(agent.task);
		}
		Pose last = agent.path.back();
		while (last.t < max_time){
			last.t += TIME_STEP;
			agent.path.push_back(last);
		}
	}
	// cout << "\n end compute soln\n";
	return agents;
}

double CBS::ComputeCost(vector<Agent> agents) {
	double max_path = 0;
	for(auto& agent:agents){
		if(agent.path.empty()) continue;
		max_path = max(agent.path.back().t, max_path);
	}		
	return max_path;
}
