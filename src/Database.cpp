#include "Database.h"

double Round_val(double in){
    return round(in*100)/100;
}

bool EmptyPose(Pose p) {
	if (p.x == -1 && p.y == -1) return true;
	return false;
}

Pose RealtoIndex(Pose p_real){
	Pose p_index;
	p_index.t = p_real.t;
	p_index.x = int(Round_val(p_real.x / SCALE_MAP_SIZE));
	p_index.y = int(Round_val(p_real.y / SCALE_MAP_SIZE));
	return p_index;
}
Pose IndextoReal(Pose p_index){
	Pose p_real;
	p_real.t = p_index.t;
	p_real.x = p_index.x * SCALE_MAP_SIZE;
	p_real.y = p_index.y * SCALE_MAP_SIZE;
	return p_real;
}

// initialization
Robot::Robot() {
	this->name = "";
	this->score = 0;
	this->cost = 0;
	this->velocity = 1;
	this->pose = *new Pose{-1,-1,0};
	this->current_task = *new Task;
	this->next_task = *new vector<Task>;
}
Task::Task() {
	this->name = "";
	this->cost = 0;
	this->done = false;
	this->initial_pose = *new Pose{-1,-1,0};
	this->final_pose = *new Pose{-1,-1,0};
	this->path = *new PATH;
}
HighLevelTask::HighLevelTask(){	
	this->name = "";
	this->score = 0;
	this->start_time = -1;
	this->end_time = -1;
	this->done = false;
}
Database::Database() {}
WorldState::WorldState() {}

// Task
Task::Task(string name, double cost, \
			vector<State> precondition, vector<State> effect, char type_,\
			bool done, Pose inital_pose, Pose final_pose) {
	this->name = name;
	this->type = type_;
	this->cost = cost;
	this->done = done;
	this->precondition = precondition;
	this->effect = effect;
	this->initial_pose = inital_pose;
	this->final_pose = final_pose;
	this->path = *new PATH;
}

void Task::SetPose(Pose initial_pose, Pose final_pose) {
	this->initial_pose = initial_pose;
	this->final_pose = final_pose;
}
void Task::SetName(string name)					{this->name = name;}
void Task::SetType(char type)					{this->type = type;}
void Task::SethltName(string name)				{this->hltname = name;}
void Task::SetPrecondtion(vector<State> pred)	{this->precondition = pred;}
void Task::SetEffect(vector<State> eff)			{this->effect =eff;}
void Task::SetPath(PATH path)					{this->path = path;}
void Task::SetCost(double cost)					{this->cost = cost;}
void Task::DoneTask(void){ this->done = true;}
void Task::UnDoneTask(void){ this->done = false;}
bool Task::CheckDone(void){ return this->done;}
string Task::GetName(void){ return this->name;}
string Task::GethltName(void){ return this->hltname;}
double Task::GetCost(void){ return this->cost;}
string Task::GetHighLevelTaskName(void){ return this->hltname;}
Pose Task::GetInitialPose(void){ return this->initial_pose;}
Pose Task::GetFinalPose(void){ return this->final_pose;}
PATH Task::GetPath(void){ return this->path;}
char Task::GetType(void){ return this->type;}
vector<State> Task::GetPrecondition(void){ return this->precondition;}
vector<State> Task::GetEffect(void){ return this->effect;}
string Task::PrintDetails(string s){
	stringstream s_out;
	s_out << s;
	s_out << this->hltname << ":"<< this->name << ":,"; 
	if(this->done){
		s_out << "(done),";
	}
	s_out <<this->type <<","<<this->initial_pose.x<<","<<this->initial_pose.y;
	s_out << ","<<this->final_pose.x <<","<<this->final_pose.y;
	s_out << ",pre:";
	for(auto& p:this->precondition){
		s_out << p.name << ":" << p.state << "," ;
	}
	s_out << ",eff:";
	for(auto& p:this->effect){
		s_out << p.name << ":" << p.state << "," ;
	}
	s_out <<"\n";
	return s_out.str();
}

// High-level Task
HighLevelTask::HighLevelTask(string name, vector<vector<Task>> action_sequence, double score){
	this->name = name;
	this->score = score;
	this->start_time = -1;
	this->end_time = -1;
	this->action_sequence = action_sequence;
	this->done = false;
}
void HighLevelTask::SetName(string name){ this->name = name;}
void HighLevelTask::SetScore(double score){ this->score = score;}
void HighLevelTask::SetTimeConstraint(double start, double end){ 
	this->start_time = start;
	this->end_time = end;
}
void HighLevelTask::SetActionSeq(vector<vector<Task>> action_sequence){this->action_sequence = action_sequence;}
void HighLevelTask::DoneTask(void){ this->done = true;}
void HighLevelTask::UnDoneTask(void){ this->done = false;}
bool HighLevelTask::CheckDone(void){ return this->done;}
string HighLevelTask::GetName(void){ 	return this->name;}
double HighLevelTask::GetScore(void){	return this->score;}
double HighLevelTask::GetStartTime(void){return this->start_time;}
double HighLevelTask::GetEndTime(void){	return this->end_time;}
vector<vector<Task>> HighLevelTask::GetActionSequence(void){ return this->action_sequence;}
void HighLevelTask::UpdateActionSequence(Task t, int k,int l){ this->action_sequence[k][l] = t; }
string HighLevelTask::PrintDetails(string s){
	stringstream s_out; 
	s_out << s;
	s_out << this->name << ": " << this->score << "\t,";
	if(this->done){
		s_out << "(done),";
	}
	for(auto a_seq:this->action_sequence){
		for(auto seq:a_seq){
			s_out << seq.GetName() << "(" << seq.GetFinalPose().x << "," << seq.GetFinalPose().y << ")";
			if(seq.CheckDone()) s_out << "(done)";
			s_out << ",";
		}
		s_out << " | ";
	}
	s_out << "\n";
	return s_out.str();
}
string HighLevelTask::PrintTaskDetails(string s){
	stringstream s_out; 
	s_out << s;
	s_out << this->name << ": ";
	if(this->done){
		s_out << "(done)";
	}
	s_out << "score:" << this->score << ", s-time:" << this->GetStartTime() << ", e-time:" << this->GetEndTime() << "\n";
	for(auto a_seq:this->action_sequence){
		for(auto seq:a_seq){
			s_out << seq.PrintDetails("");
		}
	}
	return s_out.str();
}

// Robot
Robot::Robot(string name, set<char> cap, Pose pose,\
			map<string, string> state, double score,\
			double cost, double vel){
	this->name = name;
	this->capabilities = cap;
	this->score = score;
	this->cost = cost;
	this->velocity = vel;
	this->pose = pose;
	this->state = state;
	this->current_task = Task();
}

void Robot::SetTasks(Task current_task, vector<Task> future_task){
	this->current_task = current_task;
	this->next_task = future_task;
}
void Robot::SetName(string name){this->name = name;}
void Robot::SetPose(Pose pose){this->pose = pose;}
void Robot::SetCapabilities(set<char> cap){this->capabilities = cap;}
void Robot::SetState(map<string, string> state){this->state = state;}
void Robot::UpdateState(State state, string task){
	string sym = state.state.substr(0,1);
	string val = state.state.substr(1);
	if(state.name == "obj_store"){
		if(sym == "+"){
			this->state[state.name] = this->state[state.name] + "|" + task;
		}else if(sym == "-"){
			size_t place = this->state[state.name].find("|"+task);
			if (place != std::string::npos){
				this->state[state.name].erase(place, task.length()+1);
			}
		}else if(sym == "="){
			this->state[state.name] = val;
		}
	}else{
		if(sym == ":"){
			this->state[state.name] = state.state;
		}else if(sym == "="){
			this->state[state.name] = val;
		}else if(sym == "+"){
			this->state[state.name] = to_string(int(stod(this->state[state.name]) + stod(val)));
		}else if(sym == "-"){
			this->state[state.name] = to_string(int(stod(this->state[state.name]) - stod(val)));
		}
	}
}
void Robot::SetScore(double score){this->score = score;}
void Robot::SetCost(double cost){this->cost = cost;}
void Robot::SetCurrentTask(Task curr_task){this->current_task = curr_task;}
void Robot::SetNextTasks(vector<Task> n_task){this->next_task = n_task;}
void Robot::AddNextTasks(Task n_task){this->next_task.push_back(n_task);}
void Robot::GetTasks(Task& current_task, vector<Task>& future_task){
	current_task = this->current_task;
	future_task = this->next_task;
}
Task Robot::GetCurrentTask(void){return this->current_task;}
vector<Task> Robot::GetNextTasks(void){ return this->next_task;}
string Robot::GetName(void){	return this->name;}
double Robot::GetScore(void){	return this->score;}
double Robot::GetCost(void){ 	return this->cost;}	
Pose Robot::GetPose(void){ 	return this->pose;}	
set<char> Robot::GetCapabilities(void){ return this->capabilities;}
map<string, string> Robot::GetState(void){ return this->state;}
string Robot::GetCapabilitiesString(void){
	string output = "";
	for(auto c:this->capabilities){
		output += c;
		output += ",";
	}
	return output.substr(0,output.size()-1);
}

string Robot::GetStateString(void){
	string output = "";
	for(auto s:this->state){
		output += s.first;
		output +=  ":";
		output += s.second;
		output += ",";
	}	
	return output.substr(0,output.size()-1);
}

string Robot::PrintDetails(string s){
	stringstream s_out; 
	s_out << s;
	s_out << this->name << ":(" << this->pose.x << "," << this->pose.y << "," << this->pose.t  << "):  ";
	s_out << this->score << ", " << this->cost << "|"; 
	if (this->current_task.GetName() == ""){		
		s_out << "no_task |";
	}else{		
		s_out << this->current_task.GethltName() << "|";
		s_out << this->GetCurrentTask().GetName() << " |";
	}
	s_out << "next_task_no:"<< this->next_task.size()<<"|";
	for( auto& state : this->state){
		s_out << state.first << ": " << state.second << "|";
	}
	s_out << "\n";
	return s_out.str();
}

// Compares two intervals according to staring times.
bool CompareTaskScore(HighLevelTask t1, HighLevelTask t2)
{
    return (t1.GetScore() > t2.GetScore()); // from largest to smallest
}
// World state
WorldState::WorldState(vector<Robot> &robots, vector<HighLevelTask> &tasks) {
	this->robots = robots;
	sort(tasks.begin(), tasks.end(), CompareTaskScore);
	this->tasks = tasks;
	stringstream s;
	s.str(this->PrintDetails(s.str()));
	cout << s.str();
}

void	WorldState::SetRobot(vector<Robot> robots){this->robots = robots;}
void	WorldState::SetTasks(vector<HighLevelTask> tasks){this->tasks = tasks;}
vector<Robot>			WorldState::GetRobot(void)const{return this->robots;}
vector<HighLevelTask>	WorldState::GetTasks(void)const{return this->tasks;}
vector<double> WorldState::CalculateCost(void) {
	vector<double> output = { 0,0 };
	for (auto& robot : this->robots) {
		output[0] += robot.GetScore();
		output[1] = max(robot.GetCost(), output[1]);
	}
	return output;
}

void WorldState::UpdateTask(vector<HighLevelTask> tasks){
	this->tasks.clear();
	this->tasks = tasks;
}

bool WorldState::CheckAvailableTask(void){
	if(this->tasks.size() == 0) return false;
	for(auto& t:this->tasks){
		if(t.CheckDone() == false) return true;
	}
	return false;
}

string WorldState::PrintDetails(string s){
	stringstream s_out; 
	s_out << s;
	s_out << "\nCheck all tasks\n";
    for(auto hlt:this->tasks){
        s_out.str(hlt.PrintDetails(s_out.str()));
    }
    s_out << "\nCheck all robots\n";
    for(auto r:this->robots){
        s_out.str(r.PrintDetails(s_out.str()));
    }
    s_out << "\n";
	return s_out.str();
}

// Database
Database::Database(WorldState ws, MAP map) {
	this->world_state = ws;
	this->map = map;
}

MAP Database::GetMap(void) {
	return this->map;
}

WorldState Database::GetCurrentState(void) {
	return this->world_state;
}

void Database::SetCurrentState(WorldState world_state){
	this->world_state = world_state;
}

bool Database::CheckAvailableTask(void){
	return this->world_state.CheckAvailableTask();
}