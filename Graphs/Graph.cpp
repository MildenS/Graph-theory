#include "Graph.h"
#include"Utils.h"
#include<sstream>
#include<algorithm>
#include<queue>
#include<unordered_set>



Graph::Graph(std::string fileName)
{
	std::ifstream fin(fileName);
	std::string type, info, _name;
	
	std::vector<std::string> data;
	std::string delims = " ,():";

	std::getline(fin, _name);
	this->name = _name;

	std::getline(fin, type);
	graphType = getTypeFromString(type);


	if (graphType == GraphType::Empty)
		return;

	while (std::getline(fin,info))
	{
		data = split(info, delims);
		if (isWeighted())
		{
			std::unordered_map<std::string, int64_t> nodes;
			for (uint32_t i = 1; i < data.size(); i += 2)
			{
				std::pair<std::string, uint64_t> node(data[i], std::stoi(data[i + 1]));
				nodes.insert(node);
			}
			graph.insert(std::make_pair(data[0], nodes));
		}
		else
		{
			std::unordered_map<std::string, int64_t> nodes;
			for (uint32_t i = 1; i < data.size(); ++i)
			{
				std::pair<std::string, uint64_t> node(data[i], 1);
				nodes.insert(node);
			}
			graph.insert(std::make_pair(data[0], nodes));
		}
	}

}

Graph::Graph(const Graph& rhsGraph)
{
	graphType = rhsGraph.graphType;
	graph = rhsGraph.graph;
	name = rhsGraph.name;
}

void Graph::Upload(std::string fileName)
{
	std::ofstream fout(fileName);
	fout << name + '\n';
	fout << fromTypeToString(graphType) + '\n';
	for (auto& outPair : graph)
	{
		fout << outPair.first<<": ";
		auto& nodes = outPair.second;
		for (auto& inPair : nodes)
		{
			if (isWeighted())
				fout << inPair.first + "("<< inPair.second <<"), ";
			else
				fout << inPair.first <<", ";
		}
		fout << '\n';
	}
}

std::string Graph::toString()
{
	std::string data;
	data+= fromTypeToString(graphType) + '\n';
	for (auto& outPair : graph)
	{
		data+= outPair.first + ": ";
		auto& nodes = outPair.second;
		for (auto& inPair : nodes)
		{
			if (isWeighted())
				data+= inPair.first + "(" + std::to_string(inPair.second) + "), ";
			else
				data+= inPair.first + ", ";
		}
		data+= '\n';
	}
	return data;
}

void Graph::addNode(std::string vertexName, std::unordered_map<std::string, int64_t> list)
{
	if (graph.find(vertexName) != graph.end())
	{
		throw GraphException("graph consist this vertex");
	}

	for (auto& node : list)
	{
		if (graph.find(node.first) == graph.end())
			throw GraphException("incorrent vertex adjacent list");
	}
	
	graph.insert(std::make_pair(vertexName, list));
	if (!isOriented())
	{
		for (auto& node : list)
		{
			auto it = graph.find(node.first);
			it->second.insert(std::make_pair(vertexName, node.second));
		}
	}
	


}

void Graph::addNode(std::string vertexName)
{
	if (graph.find(vertexName) != graph.end())
	{
		throw GraphException("graph already consist this vertex");
	}
	std::unordered_map<std::string, int64_t> nodes;
	graph.insert(std::make_pair(vertexName, nodes));
}

void Graph::deleteNode(std::string vertexName)
{
	auto vertexIndex = graph.find(vertexName);
	if (vertexIndex == graph.end())
	{
		throw GraphException("graph doesn't consist this vertex");
	}

	if (!isOriented())
	{
		auto adjListIt = graph.find(vertexName);
		auto& adjList = adjListIt->second;
		for (auto& pair : adjList)
		{
			auto& vertex = pair.first;
			graph.find(vertex)->second.erase(vertexName);
		}
	}
	else
	{
		for (auto& vertexes : graph)
		{
			if (vertexes.first != vertexName)
			{
				vertexes.second.erase(vertexName);
			}
		}
	}
	graph.erase(vertexIndex);
}

void Graph::addEdge(std::string firstVertex, std::string secondVertex, int64_t dist)
{
	
	if (graph.find(firstVertex) == graph.end() || graph.find(secondVertex) == graph.end())
		throw GraphException("incorrect vertexes");

	//auto graph
	auto& temp = *(graph.find(firstVertex));
	auto adjList = temp.second.find(secondVertex);
	if (adjList != temp.second.end())
		throw GraphException("you already have this edge");
	if (!isWeighted())
		dist = 1;
	graph.find(firstVertex)->second.insert(std::make_pair(secondVertex, dist));
	if (!isOriented())
		graph.find(secondVertex)->second.insert(std::make_pair(firstVertex, dist));
}

void Graph::deleteEdge(std::string firstVertex, std::string secondVertex)
{
	if (graph.find(firstVertex) == graph.end() || graph.find(secondVertex) == graph.end())
		throw GraphException("incorrect vertexes");

	graph.find(firstVertex)->second.erase(secondVertex);
	if(!isOriented())
		graph.find(secondVertex)->second.erase(firstVertex);
}


GraphType Graph::getTypeFromString(std::string type)
{
	GraphType ans;
	if (type == "WeightedOriented")
	{
		ans = GraphType::WeightedOriented;
	}
	else if (type == "UnweightedOriented")
	{
		ans = GraphType::UnweightedOriented;
	}
	else if (type == "WeightedUndirected")
	{
		ans = GraphType::WeightedUndirected;
	}
	else if (type == "UnweightedUndirected")
	{
		ans = GraphType::UnweightedUndirected;
	}
	else if (type == "Empty")
	{
		ans = GraphType::Empty;
	}
	else
		throw GraphException(std::string("invalid graph type"));
	return ans;
}

uint64_t Graph::GetDegree(std::string vertexName, uint64_t& In, uint64_t& Out)
{
	if (!isOriented())
		throw GraphException("Graph isn't oriented!!\n");
	In = Out = 0;
	uint64_t degree = 0;
	//Out = graph.count(vertexName);
	auto vertexNode = graph.find(vertexName);
	if (vertexNode == graph.end())
		throw GraphException("Graph doesn't consist this vertex!");
	Out += vertexNode->second.size();
	for (auto& lists : graph)
	{
		if (lists.first != vertexName)
		{
			if (lists.second.find(vertexName) != lists.second.end())
				++In;
		}
	}

	degree = In + Out;
	return degree;
}

bool Graph::isNeighbours(std::string firstVertex, std::string secondVertex, std::string& neighbour)
{
	if (isOriented())
		throw GraphException("Graph is oriented!!!");

	if (graph.find(firstVertex) == graph.end() || graph.find(secondVertex) == graph.end())
		throw GraphException("Incorrect vertices!!!");

	auto adjList = graph[firstVertex];
	for (auto pair : adjList)
	{
		auto searchList = *graph.find(pair.first);
		if (searchList.second.find(secondVertex) != searchList.second.end())
		{
			neighbour = pair.first;
			return true;
		}
	}
	return false;
}

uint8_t Graph::symmetricIndex = 0;

Graph Graph::symmetric_difference(const Graph& otherGraph)
{
	if (!isOriented() || !otherGraph.isOriented())
		throw GraphException("Graphs must be oriented!!!");
	if (graphType!=otherGraph.graphType)
		throw GraphException("Graphs must have the same type!!!");
	
	GraphListType tempGraph;

	//static uint8_t index;

	//merge vertexes of graphs
	for (auto pair : graph)
	{
		tempGraph.insert(std::make_pair(pair.first, AdjacentyListType()));
	}
	for (auto pair : otherGraph.graph)
	{
		tempGraph.insert(std::make_pair(pair.first, AdjacentyListType()));
	}

	//getting edges
	auto firstEdges = getEdges();
	auto secondEdges = otherGraph.getEdges();
	std::set<GraphEdge> diffEdges;
	std::set_symmetric_difference(firstEdges.begin(), firstEdges.end(),
		secondEdges.begin(), secondEdges.end(), std::inserter(diffEdges, diffEdges.begin()));

	//make graph with edges
	for (auto edge : diffEdges)
	{
		std::string firstVertex = edge.firstVertex;
		std::string secondVertex = edge.secondVertex;
		int64_t dist = edge.dist;
		auto& temp = *(tempGraph.find(firstVertex));
		auto adjList = temp.second.find(secondVertex);
		tempGraph.find(firstVertex)->second.insert(std::make_pair(secondVertex, dist));
	}

	Graph ans = Graph(std::string("symmDiff" + std::to_string(symmetricIndex)), graphType, tempGraph);
	return ans;
}

TreeType Graph::isTree(const std::string& vertexName)
{
	if (isOriented())
		throw GraphException("Graph must be undirected!!!");
	if (graph.empty())
		throw GraphException("Graph mustn't be empty!!!");
	//bfs
	std::unordered_map<std::string, bool> isViewed;
	std::unordered_map<std::string, std::string> parent;
	for (auto pair : graph)
	{
		isViewed.insert(std::make_pair(pair.first, false));
	}
	std::queue<std::string> vertexesQueue;
	vertexesQueue.push(vertexName);
	isViewed[vertexName] = true;
	while (!vertexesQueue.empty())
	{
		std::string curVertex = vertexesQueue.front();
		vertexesQueue.pop();
		for (auto v : graph[curVertex])
		{
			if (isViewed[v.first] && parent[curVertex]!=v.first) //особенность неориентированного графа
				return TreeType::HasCycle;
			isViewed[v.first] = true;
			parent[v.first] = curVertex;
			if (parent[curVertex] != v.first)
				vertexesQueue.push(v.first);
		}
	}

	for (auto pair : isViewed)
	{
		if (pair.second == false)
			return TreeType::NotLinked;
	}

	return TreeType::Tree;
}

TreeType Graph::isTree()
{
	if (isOriented())
		throw GraphException("Graph must be undirected!!!");
	if (graph.empty())
		throw GraphException("Graph mustn't be empty!!!");
	//bfs
	std::unordered_map<std::string, bool> isViewed;
	std::unordered_map<std::string, std::string> parent;
	for (auto pair : graph)
	{
		isViewed.insert(std::make_pair(pair.first, false));
	}
	std::queue<std::string> vertexesQueue;
	vertexesQueue.push(graph.begin()->first);
	isViewed[graph.begin()->first] = true;
	while (!vertexesQueue.empty())
	{
		std::string curVertex = vertexesQueue.front();
		vertexesQueue.pop();
		for (auto v : graph[curVertex])
		{
			if (isViewed[v.first] && parent[curVertex] != v.first) //особенность неориентированного графа
				return TreeType::HasCycle;
			isViewed[v.first] = true;
			parent[v.first] = curVertex;
			if (parent[curVertex] != v.first)
				vertexesQueue.push(v.first);
		}
	}

	for (auto pair : isViewed)
	{
		if (pair.second == false)
			return TreeType::NotLinked;
	}

	return TreeType::Tree;
}

TreeType Graph::isForest()
{
	if (isOriented())
		throw GraphException("Graph must be undirected!!!");
	for (auto v : graph)
	{
		if (isTree(v.first) == TreeType::HasCycle)
			return TreeType::None;
	}
	return TreeType::Forest;
}

std::vector<std::string> Graph::distancesByEdges(uint64_t k)
{
	if (graph.empty())
		throw GraphException("Graph mustn't be empty!!!");
	std::vector<std::string> ans;
	for (auto v : graph)
	{
		std::unordered_map<std::string, bool> isViewed;
		std::unordered_map<std::string, std::string> parent;
		std::unordered_map<std::string, uint64_t> distances;
		for (auto pair : graph)
		{
			distances.insert(std::make_pair(pair.first, uint64_t(-1)));
		}
		std::queue<std::string> vertexesQueue;
		vertexesQueue.push(v.first);
		isViewed[v.first] = true;
		distances[v.first] = 0;
		while (!vertexesQueue.empty())
		{
			std::string curVertex = vertexesQueue.front();
			vertexesQueue.pop();
			for (auto v : graph[curVertex])
			{
				isViewed[v.first] = true;
				parent[v.first] = curVertex;
				distances[v.first] = std::min<uint64_t>(distances[curVertex] + 1, distances[v.first]);
				if (parent[curVertex] != v.first)
					vertexesQueue.push(v.first);
			}
		}
		bool isNeeded = true;
		for (auto pair : distances)
		{
			if (pair.second > k)
			{
				isNeeded = false;
				break;
			}
		}
		if (isNeeded)
			ans.push_back(v.first);
	}


	return ans;
}

uint8_t Graph::MSTIndex = 0;

Graph Graph::MST()
{
	if (isOriented())
		throw GraphException("Graph must be undirected!!!");
	if (!isLinked())
		throw GraphException("Graph must be linked!!!");

	using primEdge = std::pair<int64_t, std::pair<std::string, std::string>>;

	GraphListType MST;
	for (const auto& pair : graph)
		MST.insert(make_pair(pair.first, AdjacentyListType()));
	std::priority_queue<primEdge, std::vector<primEdge>, std::greater<primEdge>> edges;
	std::unordered_set<std::string> visited;
	std::unordered_set<std::string> unvisited;
	//vertex, <parent, distance>
	std::unordered_map<std::string, std::pair<std::string, int64_t>> parents;

	//MST.insert(std::make_pair(graph.begin()->first, AdjacentyListType()));
	visited.insert(graph.begin()->first);
	for (const auto& pair : graph)
		if (pair.first != graph.begin()->first)
			unvisited.insert(pair.first);


	for (const auto& edge : graph[graph.begin()->first]) {
		edges.push(std::make_pair(edge.second, std::make_pair(graph.begin()->first, edge.first)));
	}

	while (!edges.empty() && !unvisited.empty() != 0)
	{
		auto edge = edges.top();
		edges.pop();
		if (visited.count(edge.second.second) == 0) {
			visited.insert(edge.second.second);
			unvisited.erase(edge.second.second);
			parents.insert(std::make_pair(edge.second.second, 
				std::make_pair(edge.second.first, edge.first)));
			for (auto& next_edge : graph[edge.second.second]) {
				if (visited.count(next_edge.first) == 0) {
					edges.push(std::make_pair(next_edge.second,
						std::make_pair(edge.second.second, next_edge.first)));
				}
			}
		}
	}

	for (const auto& edge : parents)
	{
		MST[edge.second.first].insert(make_pair(edge.first, edge.second.second));
		MST[edge.first].insert(make_pair(edge.second.first, edge.second.second));
	}
	
	Graph ans(std::string("MST") + std::to_string(MSTIndex), getGraphType(), MST);
	++MSTIndex;

	return ans;
}

std::vector<std::pair<int64_t, std::pair<std::string, std::string>>> Graph::MST_edges()
{
	if (isOriented())
		throw GraphException("Graph must be undirected!!!");
	if (!isLinked())
		throw GraphException("Graph must be linked!!!");

	using primEdge = std::pair<int64_t, std::pair<std::string, std::string>>;

	GraphListType MST;
	for (const auto& pair : graph)
		MST.insert(make_pair(pair.first, AdjacentyListType()));
	std::priority_queue<primEdge, std::vector<primEdge>, std::greater<primEdge>> edges;
	std::unordered_set<std::string> visited;
	std::unordered_set<std::string> unvisited;
	//vertex, <parent, distance>
	std::unordered_map<std::string, std::pair<std::string, int64_t>> parents;

	//MST.insert(std::make_pair(graph.begin()->first, AdjacentyListType()));
	visited.insert(graph.begin()->first);
	for (const auto& pair : graph)
		if (pair.first != graph.begin()->first)
			unvisited.insert(pair.first);


	for (const auto& edge : graph[graph.begin()->first]) {
		edges.push(std::make_pair(edge.second, std::make_pair(graph.begin()->first, edge.first)));
	}

	while (!edges.empty() && !unvisited.empty() != 0)
	{
		auto edge = edges.top();
		edges.pop();
		if (visited.count(edge.second.second) == 0) {
			visited.insert(edge.second.second);
			unvisited.erase(edge.second.second);
			parents.insert(std::make_pair(edge.second.second,
				std::make_pair(edge.second.first, edge.first)));
			for (auto& next_edge : graph[edge.second.second]) {
				if (visited.count(next_edge.first) == 0) {
					edges.push(std::make_pair(next_edge.second,
						std::make_pair(edge.second.second, next_edge.first)));
				}
			}
		}
	}

	std::vector<std::pair<int64_t, std::pair<std::string, std::string>>> ans;


	//vertex, <parent, distance>
	//std::unordered_map<std::string, std::pair<std::string, int64_t>> parents;
	for (const auto& edge : parents)
	{
		ans.push_back(std::make_pair(edge.second.second, std::make_pair(edge.first, edge.second.first)));

		//MST[edge.second.first].insert(make_pair(edge.first, edge.second.second));
		//MST[edge.first].insert(make_pair(edge.second.first, edge.second.second));
	}
	

	return ans;
}

std::pair<std::unordered_map<std::string, int64_t>, std::unordered_map<std::string, std::string>> 
Graph::Dijkstra(std::string u)
{
	if (!isWeighted())
		throw GraphException("Graph must be weighted!!!");

	if (graph.find(u) == graph.end())
		throw GraphException("Vertexes must be in graph!!!");
	
	//vertex, distance
	std::unordered_map<std::string, int64_t> d;
	//vertex, it's parent
	std::unordered_map<std::string, std::string> parents;
	using Pair = std::pair<int64_t, std::string>;

	for (const auto& pair : graph)
	{
		if (pair.first != u)
		{
			d.insert(std::make_pair(pair.first, INT64_MAX));
			parents.insert(std::make_pair(pair.first, ""));
		}
			
		else
		{
			d.insert(std::make_pair(pair.first, 0));
			parents.insert(std::make_pair(pair.first, ""));
		}
			
	}

	std::priority_queue<Pair, std::vector<Pair>, std::greater<Pair>> pq;
	pq.push(std::make_pair(0, u));

	while (!pq.empty())
	{
		auto cur_pair = pq.top();
		auto cur_v = cur_pair.second;
		auto cur_d = cur_pair.first;
		pq.pop();

		if (cur_d > d[cur_v])
		{
			continue;
		}

		for (const auto& pair : graph[cur_pair.second])
		{
			auto cur_u = pair.first;
			auto w = pair.second;
			if (d[cur_u] > d[cur_v] + w)
			{
				d[cur_u] = d[cur_v] + w;
				parents[cur_u] = cur_v;
				pq.push(std::make_pair(d[cur_u], cur_u));
			}
		}
	}

	return std::make_pair(d, parents);
}

int64_t Graph::GetEccentricity(std::string vertex)
{
	if (!isWeighted())
		throw GraphException("Graph must be weighted!!!");
	if (graph.find(vertex) == graph.end())
		throw GraphException(vertex + " not in graph!!!");

	std::unordered_map<std::string, int64_t> d;

	for (const auto& pair : graph)
	{
		if (pair.first == vertex)
			d.insert(std::make_pair(pair.first, int64_t(0)));
		else
			d.insert(std::make_pair(pair.first, INT64_MAX));
	}


	while (true)
	{
		bool flag = false;
		for (const auto& vertex : graph)
		{
			for (const auto& adjacenty : vertex.second)
			{
				if (d[vertex.first]!=INT64_MAX && adjacenty.second<INT64_MAX && 
					d[vertex.first] + adjacenty.second < d[adjacenty.first])
				{
					d[adjacenty.first] = d[vertex.first] + adjacenty.second;
					flag = true;
				}
			}
		}
		if (!flag)
			break;
	}

	int64_t ans = d.begin()->second;
	for (const auto& pair : d)
	{
		if (pair.second > ans)
			ans = pair.second;
	}

	return ans;
}

GraphListType Graph::Floyd()
{
	GraphListType ans;

	for (const auto& vertex : graph)
		ans.insert(std::make_pair(vertex.first, AdjacentyListType()));

	for (const auto& vertex : graph)
	{
		for (const auto& adj : graph)
		{
			if (vertex.first == adj.first)
				ans[vertex.first].insert(std::make_pair(adj.first, int64_t(0)));
			else
				ans[vertex.first].insert(std::make_pair(adj.first, INT64_MAX));
		}
	}

	for (const auto& vertex: graph)
		for (const auto& adj : vertex.second)
		{
			auto& vertex_adj = ans[vertex.first];
			vertex_adj[adj.first] = adj.second;
		}

	for (const auto& u: graph)
		for (const auto& i: graph)
			for (const auto& j : graph)
			{
				auto& u_adj = ans[u.first];
				auto& adj = ans[i.first];
				if (adj[u.first] <INT64_MAX && u_adj[j.first] < INT64_MAX)
					adj[j.first] = std::min(adj[j.first], adj[u.first] + u_adj[j.first]);
			}

	//TODO
	for (const auto& i: graph)
		for (const auto& j: graph)
			for (const auto& t : graph)
			{
				if (ans[i.first][t.first] < INT64_MAX && ans[t.first][t.first] < 0 &&
					ans[t.first][j.first] < INT64_MAX)
					ans[i.first][j.first] = INT64_MIN;
			}

	return ans;
}

uint64_t Graph::MaxFlow(std::string source, std::string sink)
{
	if (!isOriented())
		throw GraphException("Graph must be oriented!!!");
	if (graph.count(source) == 0 || graph.count(sink) == 0)
		throw GraphException("Both of vertexes must consists in graph!!!");

	uint64_t flow = 0;
	std::unordered_map<std::string, std::string> parents;
	for (const auto& vertex : graph)
		parents.insert(std::make_pair(vertex.first, std::string("")));

	GraphListType flow_graph = getFlowGraph();

	while (hasWay(source, sink, parents, flow_graph))
	{
		uint64_t cur_flow = uint64_t(-1);
		std::string s = sink;
		while (s != source)
		{
			const auto& adj = flow_graph[parents[s]];
			cur_flow = std::min(cur_flow, static_cast<uint64_t>(adj.at(s)));
			s = parents[s];
		}
		flow += cur_flow;
		s = sink;
		while (s != source)
		{
			std::string u = parents[s];
			flow_graph[u][s] -= cur_flow;
			flow_graph[s][u] += cur_flow;
			s = parents[s];
		}
	}

	return flow;
}

uint16_t Graph::GetMaxDistancesByEdges(std::string vertex)
{
	if (graph.empty())
		throw GraphException("Graph mustn't be empty!!!");
	int16_t ans = 0;
	auto v = *graph.find(vertex);
		std::unordered_map<std::string, bool> isViewed;
		std::unordered_map<std::string, std::string> parent;
		std::unordered_map<std::string, uint64_t> distances;
		for (auto pair : graph)
		{
			distances.insert(std::make_pair(pair.first, uint64_t(-1)));
		}
		std::queue<std::string> vertexesQueue;
		vertexesQueue.push(v.first);
		isViewed[v.first] = true;
		distances[v.first] = 0;
		while (!vertexesQueue.empty())
		{
			std::string curVertex = vertexesQueue.front();
			vertexesQueue.pop();
			for (auto v : graph[curVertex])
			{
				isViewed[v.first] = true;
				parent[v.first] = curVertex;
				distances[v.first] = std::min<uint64_t>(distances[curVertex] + 1, distances[v.first]);
				if (parent[curVertex] != v.first)
					vertexesQueue.push(v.first);
			}
		}
		bool isNeeded = true;
		for (auto pair : distances)
		{
			if (pair.second > ans)
			{
				ans = pair.second;
			}
		}

	return ans;
}



bool Graph::isLinked()
{
	std::unordered_map<std::string, bool> isViewed;
	for (const auto& pair : graph)
	{
		isViewed.insert(std::make_pair(pair.first, false));
	}
	dfs(graph.begin()->first, graph, isViewed);
	for (const auto& pair : isViewed)
	{
		if (!pair.second)
			return false;
	}
	return true;
}

std::vector<std::string> Graph::GetAllVertecies()
{
	std::vector<std::string> ans;
	for (const auto& pair : graph)
		ans.push_back(pair.first);

	return ans;
}




std::set<GraphEdge> Graph::getEdges() const
{
	std::set<GraphEdge> edges;
	std::string firstVertex, secondVertex;
	int64_t dist;
	for (auto outerPair : graph)
	{
		firstVertex = outerPair.first;
		for (auto innerPair : outerPair.second)
		{
			secondVertex = innerPair.first;
			dist = innerPair.second;
			edges.insert(GraphEdge(firstVertex, secondVertex, dist));
		}
	}
	return edges;
}

void Graph::dfs(std::string vertex, GraphListType& graph, std::unordered_map<std::string, bool>& isViewed)
{
	isViewed[vertex] = true;
	for (const auto& pair: graph[vertex])
	{
		if (!isViewed[pair.first])
			dfs(pair.first, graph, isViewed);
	}
}

bool Graph::hasWay(const std::string& source, const std::string& sink, std::unordered_map<std::string, std::string>& parents,
	GraphListType& flow_graph)
{
	if (graph.find(source) == graph.end() || graph.find(sink) == graph.end())
		throw GraphException("Both vertexes must be consists in graph!!!");

	std::unordered_map<std::string, bool> visited;
	for (const auto& vertex : graph)
		visited.insert(std::make_pair(vertex.first, false));
	visited[source] = true;

	std::queue<std::string> q;
	q.push(source);

	while (!q.empty())
	{
		std::string cur_vertex = q.front();
		q.pop();
		for (const auto& vertex : flow_graph[cur_vertex])
		{
			if (visited[vertex.first] == false && vertex.second > 0)
			{
				q.push(vertex.first);
				visited[vertex.first] = true;
				parents[vertex.first] = cur_vertex;
			}
		}
		if (cur_vertex == sink)
			return true;
	}

	return false;
}

GraphListType Graph::getFlowGraph()
{
	GraphListType ans;
	for (const auto& u : graph)
	{
		ans.insert(std::make_pair(u.first, u.second));
		auto& u_adj = ans[u.first];
		for (const auto& v : graph)
		{
			if (u_adj.find(v.first) == u_adj.end())
				u_adj.insert(std::make_pair(v.first, 0));
		}
	}

	return ans;
}


/*
WeightedOriented = 0,
	UnweightedOriented,
	WeightedUndirected,
	UnweightedUndirected*/


Graph::Graph()
{
	graphType = GraphType::Empty;
}

Graph::Graph(GraphType type)
{
	graphType = type;
}

