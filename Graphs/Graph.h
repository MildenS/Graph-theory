#pragma once
#include "CoreTypes.h"
#include <unordered_map>
#include<fstream>
#include<set>
#include<unordered_set>
//#include"Utils.h"

using GraphListType = std::unordered_map < std::string, std::unordered_map<std::string, int64_t>>;
using AdjacentyListType = std::unordered_map<std::string, int64_t>;

class Graph
{
	

public:
	//Constructors
	Graph();
	Graph(GraphType type);
	Graph(std::string fileName);
	Graph(const Graph& rhsGraph);
	Graph(std::string rhsName, GraphType rhsGraphType, const std::unordered_map < std::string, std::unordered_map<std::string, int64_t>> rhsGraph):
		graph(rhsGraph), graphType(rhsGraphType), name(rhsName) {}

	GraphType getType() { return graphType; }

	void Upload(std::string fileName);
	std::string toString();

	GraphType getGraphType() { return graphType; }
	bool isOriented() const { return (graphType == GraphType::WeightedOriented || graphType == GraphType::UnweightedOriented); }
	bool isWeighted() const { return (graphType == GraphType::WeightedOriented || graphType == GraphType::WeightedUndirected); }

	//Methods
	void addNode(std::string vertexName, std::unordered_map<std::string, int64_t> list);
	void addNode(std::string vertexName);
	void deleteNode(std::string vertexName);

	void addEdge(std::string firstVertex, std::string secondVertex, int64_t dist = 1);
	void deleteEdge(std::string firstVertex, std::string secondVertex);
	static GraphType getTypeFromString(std::string type);
	std::string GetName() const { return name; }
	bool isLinked();
	bool consistIn(std::string vertex) { return !(graph.find(vertex) == graph.end()); }
	std::vector<std::string> GetAllVertecies();


	uint64_t GetDegree(std::string vertexName, uint64_t& In, uint64_t& Out); //IA_1
	bool isNeighbours(std::string firstVertex, std::string secondVertex, std::string& neighbour); //IA_2
	Graph symmetric_difference(const Graph& otherGraph); //IB

	//II5
	TreeType isTree(const std::string& vertexName); 
	TreeType isTree();
	TreeType isForest();

	std::vector<std::string> distancesByEdges(uint64_t k); //II6


	Graph MST(); //III7
	std::vector < std::pair<int64_t, std::pair<std::string, std::string>>> MST_edges();


	std::pair<std::unordered_map<std::string, int64_t>, std::unordered_map<std::string, std::string>> 
		Dijkstra(std::string u); //IVA


	int64_t GetEccentricity(std::string vertex); //IVB


	
	GraphListType Floyd(); //IVC

	uint64_t MaxFlow(std::string source, std::string sink); //IV

	//Graphic
	AdjacentyListType GetAdjacentyListOfVertex(std::string vertex) { return graph[vertex]; }
	std::string GetFirstVertex() { return graph.begin()->first; }
	uint16_t GetVertexCount() { return graph.size(); }
	uint16_t GetMaxDistancesByEdges(std::string vertex);


private:
	
	GraphListType graph;
	GraphType graphType;
	std::string name;

	static uint8_t symmetricIndex;
	static uint8_t MSTIndex;

	std::set<GraphEdge> getEdges() const;
	void dfs(std::string vertex, GraphListType& graph, std::unordered_map<std::string, bool>& isViewed);
	bool hasWay(const std::string& source, const std::string& sink, std::unordered_map<std::string, std::string>& parents,
		GraphListType& flow_graph);
	GraphListType getFlowGraph();
	//void bfs(std::string vertexName, std::unordered_map<std::string, bool> isViewed);
};

