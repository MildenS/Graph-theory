#include "ConsoleInterface.h"
#include"Utils.h"
#include<string>
#include<iostream>
#include<cstdint>
#include<sstream>
#include<unordered_set>


void ConsoleInterface::Work()
{
	while (true)
	{
		currentMenu == generalMenu ? generalMenuWork() : tasksMenuWork();
	}


}

void ConsoleInterface::generalMenuWork()
{
	while (true)
	{
		std::cout << generalMenu;
		int command;
		std::cin >> command;
		try
		{
			switch (command)
			{
			case 1: //create
			{
				createGraph();
				break;
			}
			case 2: //addVertex
			{
				addVertex();
				break;
			}
			case 3: //AddEdge
			{
				addEdge();
				break;
			}
			case 4: //deleteVertex
			{
				deleteVertex();
				break;
			}
			case 5: //deleteEdge
			{
				deleteEdge();
				break;
			}
			case 6: //show graph
			{
				try
				{
					showGraph();
				}
				catch (...) {}
				break;
			}
			case 7: //upload
			{
				Upload();
				break;
			}
			case 8: //consoleInput
			{
				consoleInput();
				break;
			}
			case 9: //show all graphs
			{
				showAllGraphs();
				break;
			}
			case 10: //change current graph
			{
				changeCurrentGraph();
				break;
			}
			case 11: //Delete graph from graphs list
			{
				deleteGraphFromGraphList();
				break;
			}
			case 12: //Change menu
			{
				changeMenu();
				return;
				break;
			}
			}
		}
		catch (...)
		{
			std::cout << "Something has been broken :(\n";
			break;
		}
	}
}

void ConsoleInterface::tasksMenuWork()
{
	while (true)
	{
		std::cout << tasksMenu;
		int command;
		std::cin >> command;
		try
		{
			switch (command)
			{
			case 1: //IA_1
			{
				IA();
				break;
			}
			case 2: //IA_2
			{
				IA_2();
				break;
			}
			case 3: //IB
			{
				IB();
				break;
			}
			case 4: //II5
			{
				II5();
				break;
			}
			case 5: //II6
			{
				II6();
				break;
			}
			case 6: //III7
			{
				III7();
				break;
			}
			case 7: //IVA
			{
				IVA();
				break;
			}
			case 8: //IVB
			{
				IVB();
				break;
			}
			case 9: //IVC
			{
				IVC();
				break;
			}
			case 10: //V
			{
				V();
				break;
			}
			case 11: //changeMenu
			{
				changeMenu();
				return;
				break;
			}
			}
		}
		catch (...)
		{
			std::cout << "Something wrong!!!";
			break;
		}
	}
}

void ConsoleInterface::createGraph()
{
	std::cout << "Enter name of file: ";
	std::string fileName;
	std::cin >> fileName;
	//graph = std::unique_ptr<Graph>(new Graph(fileName));
	graphs.push_back(Graph(fileName));
	if (graphs.size() == 1)
		iterator = 0;
}

void ConsoleInterface::addVertex()
{
	std::cout << "Would you like enter vertex with adjacenty list or not (1 or 0): ";
	uint8_t choose;
	std::cin >> choose;
	if (int(choose))
	{
		if (graphs[iterator].isWeighted())
		{
			std::cout << "Enter vertex and it's adjacenty list over spaces:\n";
			std::string data;
			std::unordered_map<std::string, int64_t> list;
			std::cin.clear();
			std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
			std::getline(std::cin, data);
			std::stringstream sin(data);
			std::string vertexName;
			sin >> vertexName;
			int64_t curWeight;
			std::string curVertexName;
			while (sin >> curVertexName)
			{
				sin >> curWeight;
				list.insert(std::make_pair(curVertexName, curWeight));
			}
			graphs[iterator].addNode(vertexName, list);
		}
		else
		{
			std::cout << "Enter vertex and it's adjacenty list over spaces:\n";
			std::string data;
			std::unordered_map<std::string, int64_t> list;
			std::cin.clear();
			std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
			std::getline(std::cin, data);
			std::stringstream sin(data);
			std::string vertexName;
			sin >> vertexName;
			std::string curVertexName;
			while (sin >> curVertexName)
			{
				list.insert(std::make_pair(curVertexName, 1));
			}
			try
			{
				graphs[iterator].addNode(vertexName, list);
			}
			catch (GraphException& ex)
			{
				std::cout << ex.what();
			}
		}
	}
}

void ConsoleInterface::addEdge()
{
	std::cout << "Enter first vertex, second vertex and weight (if it have):\n";
	std::string firstVertex, secondVertex;
	int64_t dist = 0;
	if (graphs[iterator].isWeighted())
		std::cin >> firstVertex >> secondVertex >> dist;
	else
		std::cin >> firstVertex >> secondVertex;
	try
	{
		graphs[iterator].addEdge(firstVertex, secondVertex, dist);
	}
	catch (GraphException& ex)
	{
		std::cout << ex.what();
	}
}

void ConsoleInterface::deleteVertex()
{
	std::cout << "Enter vertex name:\n";
	std::string vertexName;
	std::cin >> vertexName;
	try
	{
		graphs[iterator].deleteNode(vertexName);
	}
	catch (GraphException& ex)
	{
		std::cout << ex.what();
	}
}

void ConsoleInterface::deleteEdge()
{
	std::cout << "Enter first and second vertexes:\n";
	std::string firstVertex, secondVertex;
	std::cin >> firstVertex >> secondVertex;
	try
	{
		graphs[iterator].deleteEdge(firstVertex, secondVertex);
	}
	catch (GraphException& ex)
	{
		std::cout << ex.what();
	}
}

void ConsoleInterface::showGraph()
{
	if (graphs.size()<1)
	{
		std::cout << "You don't entered any graph to system!\n";
		//throw std::exception();
	}
	else
	{
		std::string data = graphs[iterator].toString();
		std::cout << data << '\n';
	}
	
}

void ConsoleInterface::showCurrentGraph()
{
	if (graphs.size() == 0)
	{
		std::cout << "You don't entered any graph to system!\n";
		//throw std::exception();
	}
	else
	{
		std::string data = graphs[iterator].toString();
		std::cout << data << '\n';
	}
}

void ConsoleInterface::Upload()
{
	std::cout << "Enter name of file:\n";
	std::string fileName;
	std::cin >> fileName;
	graphs[iterator].Upload(fileName);
}

void ConsoleInterface::consoleInput()
{
	std::cout << "Enter name of graph: ";
	std::string name;
	std::cin >> name;
	std::cout << "Enter type of graph: ";
	std::string type;
	std::cin >> type;
	GraphType graphType = Graph::getTypeFromString(type);
	std::cout << "Enter number of vertexes: ";
	size_t vertexesNum;
	std::cin.clear();
	std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
	std::cin >> vertexesNum;
	std::cout << "Enter adjacenty list for every vertex (remember about graph type):\n";
	std::unordered_map<std::string, std::unordered_map<std::string, int64_t>> tempGraph;
	std::string info, delims = " ,():";
	std::cin.clear();
	std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
	for (size_t i = 0; i < vertexesNum; ++i)
	{
		std::getline(std::cin, info);
		std::vector<std::string> data = split(info, delims);
		if (graphType == GraphType::WeightedOriented || graphType == GraphType::WeightedUndirected)
		{
			std::unordered_map<std::string, int64_t> nodes;
			for (uint32_t i = 1; i < data.size(); i += 2)
			{
				std::pair<std::string, uint64_t> node(data[i], std::stoi(data[i + 1]));
				nodes.insert(node);
			}
			tempGraph.insert(std::make_pair(data[0], nodes));
		}
		else
		{
			std::unordered_map<std::string, int64_t> nodes;
			for (uint32_t i = 1; i < data.size(); ++i)
			{
				std::pair<std::string, uint64_t> node(data[i], 1);
				nodes.insert(node);
			}
			tempGraph.insert(std::make_pair(data[0], nodes));
		}

	}

	//graph.reset();
	//graph = std::unique_ptr<Graph>(new Graph(graphType, tempGraph));
	graphs.emplace_back(name, graphType, tempGraph);
}

void ConsoleInterface::showAllGraphs()
{
	if (graphs.size() == 0)
	{
		std::cout << "You don't entered any graph to system!\n";
		//throw std::exception();
	}
	else
	{
		std::string data;
		for (size_t i = 0; i < graphs.size(); ++i)
		{
			data += std::to_string(i + 1) + ". " + graphs[i].GetName() + " (" + 
				fromTypeToString(graphs[i].getType()) + ").\n";
		}
		std::cout << data;
	}

}

void ConsoleInterface::changeCurrentGraph()
{
	std::cout << "Enter name of new current graph:\n";
	std::cin.clear();
	std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
	std::string newCurrentName;
	std::cin >> newCurrentName;
	int index = 0;
	auto it = std::find_if(graphs.begin(), graphs.end(), [&](Graph& graph)
		{
			if (graph.GetName() != newCurrentName)
				++index;
			return graph.GetName() == newCurrentName;
		});
	if (it == graphs.end())
	{
		std::cout << "Incorrect new current graph name!!!\n";
		return;
	}
	else
	{
		iterator = index;
	}
}

void ConsoleInterface::deleteGraphFromGraphList()
{
	std::cout << "Enter name of deleted graph:\n";
	std::cin.clear();
	std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
	std::string deletedName;
	std::cin >> deletedName;
	deleteByValue(graphs, deletedName);
}

void ConsoleInterface::changeMenu()
{
	if (currentMenu == tasksMenu)
		currentMenu = generalMenu;
	else
		currentMenu = tasksMenu;
	//currentMenu = (currentMenu == tasksMenu ? generalMenu : tasksMenu);
}

void ConsoleInterface::IA()
{
	std::cout << "Enter vertex name: ";
	std::cin.clear();
	std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
	std::string vertexName;
	std::cin >> vertexName;
	uint64_t in, out, degree;
	try
	{
		degree = graphs[iterator].GetDegree(vertexName, in, out);
		std::cout << "Dergree = " << degree << " InDegree = " << in << " OutDegree = " << out << '\n';
	}
	catch (GraphException& ex)
	{
		std::cout << ex.what() + '\n';
	}
}

void ConsoleInterface::IA_2()
{
	std::cout << "Enter vertices names:\n";
	std::cin.clear();
	std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
	std::string firstVertex, secondVertex, neighbour;
	std::cin >> firstVertex>>secondVertex;
	try
	{
		bool isNeighbours = graphs[iterator].isNeighbours(firstVertex, secondVertex, neighbour);
		if (isNeighbours)
			std::cout << "\n" << firstVertex << " and " << secondVertex << " is neighbours over " << neighbour<<'\n';
		else
			std::cout << "\n" << firstVertex << " and " << secondVertex << " isn't neighbours over 1 vertex\n";
	}
	catch (GraphException& ex)
	{
		std::cout << ex.what() + "\n";
	}

}

void ConsoleInterface::IB()
{
	std::cout << "Enter name of second graph: ";
	std::string secondGraph;
	std::cin.clear();
	std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
	std::cin >> secondGraph;
	int index = 0;
	auto it = std::find_if(graphs.begin(), graphs.end(), [&](Graph& graph)
		{
			if (graph.GetName() != secondGraph)
				++index;
			return graph.GetName() == secondGraph;
		});
	if (it == graphs.end())
	{
		std::cout << "Incorrect second current graph name!!!\n";
		return;
	}
	try
	{
		Graph diffGraph = graphs[iterator].symmetric_difference(graphs[index]);
		graphs.push_back(diffGraph);
		std::cout << "Graph of symmetric difference added to graphs list!\n";
	}
	catch (const GraphException& ex)
	{
		std::cout << ex.what() + "\n";
	}
}

void ConsoleInterface::II5()
{
	try
	{
		if (graphs[iterator].isTree() == TreeType::Tree)
			std::cout << "Graph is a tree!\n";
		else if (graphs[iterator].isForest() == TreeType::Forest)
			std::cout << "Graph is a forest!\n";
		else
			std::cout << "Graph isn't tree or forest:(\n";
	}
	catch (const GraphException& ex)
	{
		std::cout << ex.what() + "\n";
	}
}

void ConsoleInterface::II6()
{
	std::cout << "Enter k: ";
	uint64_t k;
	std::cin.clear();
	std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
	std::cin >> k;
	if (k < 0)
	{
		std::cout << "k must be > 0!!!\n";
		return;
	}
	try 
	{
		std::vector<std::string> ans = graphs[iterator].distancesByEdges(k);
		if (ans.empty())
		{
			std::cout << "There aren't vertexes with this k:(\n";
			return;
		}
		for (auto str : ans)
			std::cout << str + ", ";
		std::cout << '\n';
	}
	catch (const GraphException& ex)
	{
		std::cout << ex.what() + "\n";
	}
}

void ConsoleInterface::III7()
{
	try
	{
		graphs.push_back(graphs[iterator].MST());
	}
	catch (const GraphException& ex)
	{
		std::cout << ex.what() + "\n";
	}
}

void ConsoleInterface::IVA()
{
	std::cout << "Enter u, v1 and v2:\n";
	std::string u, v1, v2;
	std::cin >> u >> v1 >> v2;
	if (!graphs[iterator].consistIn(u) || !graphs[iterator].consistIn(v1) ||
		!graphs[iterator].consistIn(v2))
	{
		std::cout << "All vertexes must be in current graph\n";
		return;
	}
	try
	{
		auto pair = graphs[iterator].Dijkstra(u);
		const auto& d = pair.first;
		const auto& parents = pair.second;
		
		std::vector<std::string> ans;
		if (d.at(v1) < INT64_MAX)
		{
			std::cout << "Distance from u to v1 = " + std::to_string(d.at(v1)) + "\n";
			std::string v = v1;
			while (v != u) {
				ans.push_back(v);
				v = parents.at(v);
			}
			ans.push_back(u);
			std::reverse(ans.begin(), ans.end());
			std::string way = "Way from u to v1 is :";
			for (const auto& str : ans)
				way += str + " ";
			std::cout << way + "\n";

		}
		else
		{
			std::cout << "There aren't way from " << u << " to " << v1;
		}
		if (d.at(v2) < INT64_MAX)
		{
			ans.clear();
			std::cout << "Distance from u to v2 = " + std::to_string(d.at(v2)) + "\n";
			std::string v = v2;
			while (v != u) {
				ans.push_back(v);
				v = parents.at(v);
			}
			ans.push_back(u);
			std::reverse(ans.begin(), ans.end());
			std::string way = "Way from u to v2 is :";
			for (const auto& str : ans)
				way += str + " ";
			std::cout << way + "\n";

		}
		else
		{
			std::cout << "There aren't way from " << u << " to " << v2;
		}

	}
	catch (const GraphException& ex)
	{
		std::cout << ex.what() + "\n";
		return;
	}
}

void ConsoleInterface::IVB()
{
	try
	{
		std::vector<std::string> vertexes = graphs[iterator].GetAllVertecies();
		std::unordered_map<std::string, int64_t> eccentricities;
		for (const auto& vertex : vertexes)
		{
			int64_t cur_d = graphs[iterator].GetEccentricity(vertex);
			eccentricities.insert(std::make_pair(vertex, cur_d));
			std::cout << vertex << " " << cur_d << "\n";
		}

		std::unordered_set<std::string> graph_radius;
		int64_t cur_radius = eccentricities.begin()->second;
		for (const auto& pair : eccentricities)
		{
			if (pair.second == cur_radius)
			{
				graph_radius.insert(pair.first);
			}
			else if (pair.second < cur_radius)
			{
				cur_radius = pair.second;
				graph_radius.clear();
				graph_radius.insert(pair.first);
			}
			
		}
		std::string ans = "Radius of graph: ";
		for (const auto& vertex : graph_radius)
			ans += vertex + " ";
		std::cout << ans + "\n";
	}
	catch (const GraphException& ex)
	{
		std::cout << ex.what() + "\n";
		return;
	}



}

void ConsoleInterface::IVC()
{
	try
	{
		auto distances = graphs[iterator].Floyd();
		std::cout << "Distances between all pair of vertexes: \n";
		for (const auto& vertex : distances)
		{
			std::cout << vertex.first << ": ";
			for (const auto& adj : vertex.second)
			{
				if (adj.second < INT64_MAX && adj.second > INT64_MIN)
					std::cout << adj.first << " (" << adj.second << "), ";
				else if (adj.second == INT64_MAX)
					std::cout << adj.first << " (inf), ";
				else
					std::cout << adj.first << " (-inf), ";
			}
			std::cout << "\n";
		}
		std::cout << "\n";
	}
	catch (const GraphException& ex)
	{
		std::cout << ex.what() + "\n";
	}
}

void ConsoleInterface::V()
{
	std::string source, sink;
	std::cout << "Enter source and sink:\n";
	std::cin >> source >> sink;
	try 
	{
		auto flow = graphs[iterator].MaxFlow(source, sink);
		std::cout << "Max flow = " << flow << "\n";
	}
	catch (const GraphException& ex)
	{
		std::cout << ex.what() + "\n";
	}
}
