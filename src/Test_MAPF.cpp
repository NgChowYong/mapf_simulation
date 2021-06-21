// # CBS for test
// # Author: Ng Chow Yong
// # Date: 2020-12-20 begin
// cpp
#include <sstream>
#include <string>
#include <stdlib.h>     /* srand, rand */
// roscpp and ros msgs used
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "nav_msgs/Path.h"
#include "nav_msgs/OccupancyGrid.h"
#include "geometry_msgs/PoseStamped.h"
#include "multi_robot_msgs/RobotState.h"      
#include "multi_robot_msgs/RobotCommand.h"
#include "multi_robot_msgs/Task.h"
// my class
#include "Database.h"
#include "HelpFunc.h"
#include "AStar.h"
#include "CBS.h"
#include "ScenarioLoader.h"

// from owner code
#include <stdint.h>
#include <numeric>
#include <algorithm>

using namespace std;

ros::Time start_time;

void PrintPathSolution(vector<Agent>);
void LoadMap(const char* fname, MAP &map, int &width, int &height);

void StateCallback(const multi_robot_msgs::RobotState::ConstPtr& msg){
    // print result ?
}

multi_robot_msgs::Task AssignTask(Task t, ros::Time time){
    multi_robot_msgs::Task task;
    task.header.stamp = ros::Time::now();
    task.name = t.GetName();
    task.initial_pose.position.x = IndextoReal(t.GetInitialPose()).x;
    task.initial_pose.position.y = IndextoReal(t.GetInitialPose()).y;
    task.final_pose.position.x = IndextoReal(t.GetFinalPose()).x;
    task.final_pose.position.y = IndextoReal(t.GetFinalPose()).y;
    task.action = t.GetName();
    task.path = *new nav_msgs::Path;
    for(auto& pos:t.GetPath()){
        geometry_msgs::PoseStamped ps;
        ps.header.stamp = start_time;
        ps.header.stamp.sec += pos.t;
        ps.pose.position.x = IndextoReal(pos).x;
        ps.pose.position.y = IndextoReal(pos).y;
        task.path.poses.push_back(ps);
    }
    return task;
}

void publish_path(vector<Agent> robots, ros::Publisher pub_path){
    // cout << "start publish " << start_time.sec << "\n";
    for(auto& robot:robots){
        Robot r = robot.robot;
        multi_robot_msgs::RobotCommand command;
        command.robot_name = r.GetName();
        command.current_task = AssignTask(robot.task[0], start_time);
        pub_path.publish(command);
    }
}

void ConvertScenToCBS(ScenarioLoader scen, vector<Agent>& agents){
    if(agents.size() > scen.GetNumExperiments()) {
        cout << "NUMBER OF AGENTS TOO MUCH !!!\n";
        return;
    }
    Task t;
    t.SetName("task");
    vector<Task> tv;
    tv.push_back(t);
    for(int i = 0; i<agents.size(); i++){
        Pose ps = {scen.GetNthExperiment(i).GetStartX(),scen.GetNthExperiment(i).GetStartY()};
        Pose pe = {scen.GetNthExperiment(i).GetGoalX(),scen.GetNthExperiment(i).GetGoalY()};
        agents[i].robot.SetPose(ps);
        t.SetPose(ps,pe);
        agents[i].task = tv;
        agents[i].target.push_back(make_pair(ps,pe));
    }
}

int main(int argc, char **argv){
    ros::init(argc, argv, "MAPF_allocation");
    ros::NodeHandle n;
    ros::Publisher pub_path = n.advertise<multi_robot_msgs::RobotCommand>("/robot_command", 50);
    ros::Publisher pub_map = n.advertise<nav_msgs::OccupancyGrid>("/map", 50);
    ros::Subscriber sub_state = n.subscribe("/robot_states", 50, StateCallback);

    MAP mapData;
    int width, height;
    string map_file,scen_file;
    int robot_no;

    // get parameter from launch file
    n.getParam("/map_file", map_file);
    n.getParam("/scen_file", scen_file);
    n.getParam("/robot_no", robot_no);
    cout << "get param done\n";

    // load map and scene
    LoadMap(map_file.c_str(), mapData, width, height);
    cout << "load map done\n";
    ScenarioLoader scen(scen_file.c_str());
    cout << "load scen done\n";

    vector<Agent> solution;
    set<char> c;
    for(int i =1;i<=robot_no;i++){
        Agent a;
        Robot r;
        r.SetName("robot"+to_string(i));
        a.robot = r;
        solution.push_back(a);
    }
    ConvertScenToCBS(scen, solution);
    cout << "Conversion done\n";

    // path finding planner 
    cout << "waiting for path precompute\n";
    AStar astar(mapData);    
    CBS cbs(astar);
    cout << "Initialize done\n";

    // cbs path search
    ROS_INFO_STREAM("start compute CBS");
    solution = cbs.Solve(solution);
    PrintPathSolution(solution);
    cout << "Solve CBS done\n";
    while (ros::ok() && ros::Time::now().sec == 0){ }
    ros::Duration(2).sleep();
    start_time = ros::Time::now();
    publish_path(solution, pub_path);

    ros::Rate loop_rate(1);
    while (ros::ok()){
        nav_msgs::OccupancyGrid map;
        map.header.stamp =  ros::Time::now();
        map.header.frame_id = "world";
        map.info.map_load_time = ros::Time::now();
        map.info.resolution = SCALE_MAP_SIZE;
        map.info.width = width;
        map.info.height = height;
        map.info.origin.position.x = 0.0;
        map.info.origin.position.y = 0.0;
        for(int y =0;y<mapData.size();y++){
            for(auto x:mapData[y]){
                if(x){
                    map.data.push_back(255);
                }else{
                    map.data.push_back(0);
                }
            }
        }
        pub_map.publish(map);
        loop_rate.sleep();
        ros::spinOnce();
    }
    cout << "\nend computing\n";
    return 0;
}

void PrintPathSolution(vector<Agent> agents){
    stringstream s_out;
    s_out << "\nPath Solution: \n";
	for(auto& a:agents){
		s_out << a.robot.GetName() << ":\n";
        for(auto& p:a.path){
			s_out << "(" << p.x << "," << p.y << "," << p.t << "),";
		}
		s_out << "\n";
	}
    ROS_INFO_STREAM(s_out.str());
}

void LoadMap(const char* fname, MAP &map, int &width, int &height)
{
	FILE *f;
	f = fopen(fname, "r");
	if (f)
    {
		fscanf(f, "type octile\nheight %d\nwidth %d\nmap\n", &height, &width);
		for (int y = 0; y < height; y++)
		{
            vector<bool> vb;
			for (int x = 0; x < width; x++)
			{
				char c;
				do {
					fscanf(f, "%c", &c);
				} while (isspace(c));
                if(c == '.' || c == 'G' || c == 'S'){vb.push_back(true);}
                else {vb.push_back(false);}
			}
            map.push_back(vb);
			//printf("\n");
		}
		fclose(f);
    }
}
