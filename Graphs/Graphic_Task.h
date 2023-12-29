#pragma once
#include "Graph.h"
#include<unordered_map>
#include<queue>

class Graph;

class Graphic_Task
{
	struct Node
	{
		float x, y;
		std::string name;
		std::unordered_map<std::string, int64_t> adj;
		bool is_ready = false;
		float radius = 0;
	};


	enum class TypeOfGraph :uint8_t
	{
		Graph,
		MST
	};

public:

	Graphic_Task(std::string graph_path);
	


	void Work();



private:
	std::unique_ptr<Graph> graph;
	Graph MST;
	TypeOfGraph current_graph;
};

