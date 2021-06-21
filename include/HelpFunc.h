#pragma once

#include "Database.h"

string SeparateString(string &, string);
void LoadTaskFromFile(string, vector<State>&, vector<Task>&);
vector<HighLevelTask> LoadHLTaskFromFile(string, vector<Task>&);
vector<Robot> LoadRobotFromFile(string, vector<State>&);
MAP LoadMapFromFile(string);
void SavePrecomputeToFile(string, unordered_map<string, int>);
unordered_map<string, int> LoadPrecomputeFromFile(string);

