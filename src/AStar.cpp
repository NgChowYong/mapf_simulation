#include "AStar.h"

string HASH_POSE(Pose p1, Pose p2){
	return to_string((int)p1.x) + "," + to_string((int)p1.y) + "," + to_string((int)p2.x) + "," + to_string((int)p2.y);
}

AStarNode::AStarNode(void) {}
AStarNode::AStarNode(Pose p) {
    this->state = p;
    this->f = 0;
    this->g = 0;
    this->h = 0;
    this->parent = nullptr;
}
AStarNode::AStarNode(const AStarNode& other) {
	this->f = other.f;
	this->g = other.g;
	this->h = other.h;
	this->state = other.state;
	if (other.parent == nullptr) {
		this->parent = nullptr;
	}
	else {
		this->parent = new AStarNode(*other.parent);
	}
}


AStar::AStar(void) {}
AStar::AStar(MAP map) {
	this->map = map;    
}

double AStar::Heuristic(AStarNode node, AStarNode goal) {
    return 1.001 * (fabs(node.state.x - goal.state.x) + fabs(node.state.y - goal.state.y));
}

bool AStar::CheckClosedSet(AStarNode n , unordered_set<AStarNode> s) {
    for (auto& set_node : s) {
        if (set_node.state.x == n.state.x && set_node.state.y == n.state.y && set_node.state.t == n.state.t) return true;
    }
    //return (s.find(n) != s.end());
    return false;
}

vector<Pose> AStar::GetNewState(Pose p) {
    vector<Pose> output;
    Pose p_new;
    int col = this->map.size();
    int row = this->map[0].size();
    if (p.x + 1 < row && this->map[int(p.y)][int(p.x + 1)] != 'x') {
        p_new.x = p.x + 1;
        p_new.y = p.y;
        p_new.t = p.t + TIME_STEP;
        output.push_back(p_new);
    }
    if (p.y + 1 < col && this->map[int(p.y + 1)][int(p.x)] != 'x') {
        p_new.x = p.x ;
        p_new.y = p.y + 1;
        p_new.t = p.t + TIME_STEP;
        output.push_back(p_new);
    }
    if (p.x - 1 >= 0 && this->map[int(p.y)][int(p.x - 1)] != 'x') {
        p_new.x = p.x - 1;
        p_new.y = p.y;
        p_new.t = p.t + TIME_STEP;
        output.push_back(p_new);
    }
    if (p.y - 1 >= 0 && this->map[int(p.y - 1)][int(p.x)] != 'x') {
        p_new.x = p.x;
        p_new.y = p.y - 1;
        p_new.t = p.t + TIME_STEP;
        output.push_back(p_new);
    }    
    p_new = *new Pose(); // memory leak !!!
    p_new.x = p.x;
    p_new.y = p.y;
    p_new.t = p.t + TIME_STEP;
    output.push_back(p_new);
    return output;
}

bool AStar::CheckGoal(AStarNode state, AStarNode goal) {
    return (int(state.state.x) == int(goal.state.x)) && (int(state.state.y) == int(goal.state.y));
}

vector<Pose> AStar::ReconstructPlan(AStarNode node) {
    vector<Pose> output;
    while (node.parent != nullptr) {
        Pose temp{node.state.x, node.state.y, node.state.t };
        output.push_back(temp);
        node = *node.parent;
    }
    reverse(output.begin(), output.end());
    return output;
}

vector<Pose> AStar::Solve(Pose initial, Pose goal_, PATH conflict) {
    AStarNode initial_node(initial);
    AStarNode goal_node(goal_);

    priority_queue<AStarNode, vector<AStarNode>, AStarcomparator> open_set;
    unordered_set<AStarNode> closed_set;
    open_set.push(initial_node);
    AStarNode current_node = open_set.top();

    while (!open_set.empty()) {
        current_node = open_set.top();
        open_set.pop();
        if (CheckGoal(current_node, goal_node)) {return ReconstructPlan(current_node);}
        closed_set.insert(current_node);
        for (auto& possible_act : GetNewState(current_node.state)) {
            AStarNode New_node(possible_act);
            if (CheckClosedSet(New_node, closed_set)) continue;
            if (CheckConflict(New_node, conflict)) continue;
            New_node.g = current_node.g + 1;
            New_node.h = Heuristic(New_node, goal_node);
            New_node.f = New_node.g + New_node.h;
            New_node.parent = &current_node;
            open_set.push(New_node);
        }
    }
    cout << "pose error: "<< initial.x << "," << initial.y << ":" << goal_.x << "," << goal_.y << "\n";
    throw std::invalid_argument("cannot find path !!! ");
}

bool AStar::CheckConflict(AStarNode New_node, PATH conflict) {
    for (auto& conflict_pose : conflict) {
        if (New_node.state.x == conflict_pose.x &&\
            New_node.state.y == conflict_pose.y &&\
            (abs(New_node.state.t - conflict_pose.t) < TIME_STEP) ){
            return true;
        }
    }
    return false;
}

unordered_map<string, int> AStar::GetPathData(WorldState ws) {
    unordered_map<string, int> output;
    unordered_set<Pose> vec_p;
    for (auto& robot : ws.GetRobot()) {
        vec_p.insert(RealtoIndex(robot.GetPose()));
    }
    for (auto& task : ws.GetTasks()) {
        for (auto& act_ : task.GetActionSequence()) {
            for (auto& act : act_) {
                vec_p.insert(act.GetInitialPose());
                vec_p.insert(act.GetFinalPose());
            }
        }
    }
    cout << "checked all available pose: " << vec_p.size() << "\n";
    for (auto& p1 : vec_p) {
        for (auto& p2 : vec_p) {
            if (p1.x == p2.x && p1.y == p2.y) continue;
            if (EmptyPose(p1) || EmptyPose(p2)) continue;
            // cout << "finding path: " << p1.x << "," << p1.y << ":" << p2.x << "," << p2.y << "\n";
            vector<Pose> path = Solve(p1, p2);
            output[HASH_POSE(p1,p2)] = path.back().t;
        }
    }
    cout << "done precompute path length\n";
    return output;
}