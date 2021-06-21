#include "HelpFunc.h"

// some function for load task
string SeparateString(string &line, string delimiter){
	size_t l = line.find(delimiter);
	string name = line.substr(0,l);
	if (l == -1){
		line = "";
	}
	line = line.substr(l+1);
	return name;
}

vector<State> GetPrecondEff(vector<State> state, string &line, string delimiter1, string delimiter2){
	std::string delimiter3 = "|";

	vector<State> precondeff;
	string precondition = SeparateString(line, delimiter2);
	precondition = precondition.substr(1,precondition.size()-2);
	
	string state_pre = SeparateString(precondition, delimiter3);
	while(state_pre != ""){
		// consider only one precondition or effect
		State s;
		if(state_pre == ""){
			s.name = "";
			s.state = "";
		}else{
			s.name = state_pre.substr(0,state_pre.size()-2);
			s.state = state_pre.substr(state_pre.size()-2);
		}
		precondeff.push_back(s);
		state_pre = SeparateString(precondition, delimiter3);
	}
	// bool found = false;
	// for(auto st:state){
	// 	if(s.name == st.name || s.name == "") {
	// 		found = true;
	// 		break;
	// 	}
	// }
	// if (found == false){
	// 	string str_ = " found unset parameter " + s.name ;
	// 	throw std::invalid_argument(str_);
	// }
	return precondeff;
}

// state and task
void LoadTaskFromFile(string file_name,vector<State> &state, vector<Task> &task){
	state.clear();
	task.clear();
	cout << "load task\n";
	ifstream task_file(file_name);
	int type = 0;
	if (task_file.is_open()) {
		string line;
		std::string delimiter1 = ":";
		std::string delimiter2 = ",";
		while (getline(task_file, line)) {
			if(line == "STATE") {type = 0; continue;}
			if(line == "TASK") {type = 1; continue;}
			if(type == 0){
				State s;
				s.name = SeparateString(line, delimiter2);
				s.state = SeparateString(line, delimiter2);
				if( s.state == "x") s.state = "";
				state.push_back(s);
			}else if(type == 1){
				Task t;
				t.SetName(SeparateString(line, delimiter1));
				t.SetCost(stod(SeparateString(line, delimiter2)));
				t.SetType(SeparateString(line, delimiter2)[0]);
				t.SetPrecondtion(GetPrecondEff(state ,line, delimiter1, delimiter2));
				t.SetEffect(GetPrecondEff(state, line, delimiter1, delimiter2));
				task.push_back(t);
			}
		}
		task_file.close();
	}else{
		cout << "cannot open: " << file_name << "\n";
		throw std::invalid_argument("cannot open file !!! ");
	}
}

void precond_eff(Task &t, HighLevelTask HLT){
	vector<State> state_t = t.GetPrecondition();
	for(auto& temp_s:state_t){
		if(temp_s.name == "obj_store"){
			temp_s.state = HLT.GetName();
		}
	}
	t.SetPrecondtion(state_t);	
	state_t = t.GetEffect();
	for(auto& temp_s:state_t){
		if(temp_s.name == "obj_store"){
			if(t.GetName() == "load"){
				temp_s.state = "+"+HLT.GetName();
			}else if(t.GetName() == "unload"){
				temp_s.state = "-"+HLT.GetName();
			}
		}
	}
	t.SetEffect(state_t);				
}

// High level task
vector<HighLevelTask> LoadHLTaskFromFile(string file_name, vector<Task>& task){
	cout << "load high level task\n";
	vector<HighLevelTask> HLT_vec;
	ifstream task_file(file_name);
	if (task_file.is_open()) {	
		string line;
		std::string delimiter = ",";
		int i = 0;
		while (getline(task_file, line)) {
			if (i == 0) {
				i++;
				continue;
			}
			HighLevelTask HLT;
			HLT.SetName(SeparateString(line, delimiter));
			HLT.SetScore(stod(SeparateString(line, delimiter)));
			double t_s = stod(SeparateString(line, delimiter));
			double t_e = stod(SeparateString(line, delimiter));
			HLT.SetTimeConstraint(t_s, t_e);			
			double xs = stod(SeparateString(line, delimiter));
			double ys = stod(SeparateString(line, delimiter));
			Pose ps; ps.x = xs; ps.y=ys;ps.t = 0;
			double xe = stod(SeparateString(line, delimiter));
			double ye = stod(SeparateString(line, delimiter));
			Pose pe; pe.x = xe; pe.y=ye;pe.t = 0;
			double xe2 = stod(SeparateString(line, delimiter));
			double ye2 = stod(SeparateString(line, delimiter));
			Pose pe2; pe2.x = xe2; pe2.y=ye2;pe2.t = 0;
			vector<vector<Task>> temp;
			if (EmptyPose(ps)){
				vector<Task> t1;
				Task t = task[2];
				t.SethltName(HLT.GetName());
				t.SetPose(ps,pe);
				precond_eff(t, HLT);
				t1.push_back(t);
				temp.push_back(t1);
			}else{
				vector<Task> t1,t2;
				Task t = task[0];
				t.SethltName(HLT.GetName());
				t.SetPose(ps,ps);
				precond_eff(t, HLT);
				t1.push_back(t);
				precond_eff(t, HLT);
				t = task[1];
				precond_eff(t, HLT);
				t.SethltName(HLT.GetName());
				t.SetPose(ps,pe);
				t2.push_back(t);
				t.SetPose(ps,pe2);
				t2.push_back(t);
				temp.push_back(t1);
				temp.push_back(t2);
			}
			HLT.SetActionSeq(temp);
			HLT_vec.push_back(HLT);		
		}
		task_file.close();
	}else{
		cout << "cannot open: " << file_name << "\n";
		throw std::invalid_argument("cannot open file !!! ");
	}

	return HLT_vec;
}

// robot
vector<Robot> LoadRobotFromFile(string file_name, vector<State>& state){
	cout << "load robot task\n";
	vector<Robot> r_vec;
	ifstream robot_file(file_name);
	if (robot_file.is_open()) {
		string line;
		std::string delimiter1 = ":";
		std::string delimiter2 = ",";
		std::string delimiter3 = "|";
		int i = 0;
		while (getline(robot_file, line)) {
			if (i == 0) {
				i++;
				continue;
			}
			Robot r;
			r.SetName(SeparateString(line, delimiter1));
			string s_temp = SeparateString(line, delimiter3);
			s_temp = s_temp.substr(1, s_temp.size()-2);
			double x = stod(SeparateString(s_temp, delimiter2));
			double y = stod(SeparateString(s_temp, delimiter2));
			Pose p; p.x = x; p.y = y; p.t = 0; r.SetPose(p);
			
			s_temp = SeparateString(line, delimiter3);
			s_temp = s_temp.substr(1, s_temp.size()-2);
			set<char> sc; sc.insert(SeparateString(s_temp, delimiter2)[0]);
			sc.insert(SeparateString(s_temp, delimiter2)[0]);
			r.SetCapabilities(sc);

			s_temp = SeparateString(line, delimiter3);
			s_temp = s_temp.substr(1, s_temp.size()-2);
			map<string, string> m; 
			string s_temp2 = SeparateString(s_temp, delimiter2);
			string state_name = SeparateString(s_temp2, delimiter1);
			string state_val = SeparateString(s_temp2, delimiter1);
			m[state_name] = state_val;
			s_temp2 = SeparateString(s_temp, delimiter2);
			state_name = SeparateString(s_temp2, delimiter1);
			state_val = SeparateString(s_temp2, delimiter1);
			m[state_name] = state_val;
			s_temp2 = SeparateString(s_temp, delimiter2);
			state_name = SeparateString(s_temp2, delimiter1);
			state_val = SeparateString(s_temp2, delimiter1);
			if(state_val == "x"){ m[state_name] = "";}
			else{  m[state_name] = state_val;}
			r.SetState(m);
			r_vec.push_back(r);
		}
		robot_file.close();
	}else{
		cout << "cannot open: " << file_name << "\n";
		throw std::invalid_argument("cannot open file !!! ");
	}
	return r_vec;
}

// map
MAP LoadMapFromFile(string file_name){
	MAP m;
	ifstream map_file(file_name);
	if (map_file.is_open()) {
		string line;
		while (getline(map_file, line)) {
			vector<bool> char_vec;
			for (auto& l : line) {
				bool wall = false;
				if(l == '.' || l == 'G' || l == 'S'|| l == 'x'){
					wall = true;
				}
				char_vec.push_back(wall);
			}
			m.push_back(char_vec);
		}
		reverse(m.begin(), m.end());
		map_file.close();
	}else{
		cout << "cannot open: " << file_name << "\n";
		throw std::invalid_argument("cannot open file !!! ");
	}
	return m;
}

// save and load file
void SavePrecomputeToFile(string file_name, unordered_map<string, int> pre) {
	ofstream outputfile(file_name);
	if (outputfile.is_open())
	{
		for (auto& p : pre) {
			outputfile << p.first << "||" << p.second << "\n";
		}
	    outputfile.close();
	}
	else {
	    cout << "Unable to open file";
	    throw std::invalid_argument("cannot open file !!! ");
	}

}

unordered_map<string, int> LoadPrecomputeFromFile(string file_name) {
	unordered_map<string, int> output;
	ifstream map_file(file_name);
	if (map_file.is_open()) {
		string line;
		string delimiter = "||";
		string token;
		while (getline(map_file, line)) {
			//cout << "next line: \n";
			size_t pos = line.find(delimiter);
			token = line.substr(0, pos);
			line.erase(0, pos + delimiter.length());
			output[token] = stoi(line);
		}
		map_file.close();
	}else {
		cout << "cannot open: " << file_name << "\n";
		throw std::invalid_argument("cannot open file !!! ");
	}

	return output;
}