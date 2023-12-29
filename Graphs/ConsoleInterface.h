#pragma once
#include"Graph.h"
//#include"Utils.h"
#include<string>
#include<memory>

class ConsoleInterface
{

	//TODO: Tasks menu Work

public:

	ConsoleInterface() { iterator = 0; currentMenu = generalMenu; }
	void Work();

private:

	std::unique_ptr<Graph> graph;
	std::vector<Graph> graphs;
	uint32_t iterator;
	std::string currentMenu = generalMenu;

	std::string menu = std::string("Welcome to Graph Editor! :)\n") +
		std::string("1. Create Graph with file data.\n") +
		std::string("2. Add vertex to graph.\n") +
		std::string("3. Add edge to graph.\n") +
		std::string("4. Delete vertex from graph.\n") +
		std::string("5. Delete edge from graph.\n") +
		std::string("6. Show current graph.\n") +
		std::string("7. Upload graph.\n") +
		std::string("8. Create graph with console.\n") +
		std::string("9. IA. Degree oriented graph's vertex.\n") +
		std::string("10. IA_2, Neighbours over 1 vertex.\n") +
		std::string("Enter number of command.\n");

	std::string generalMenu = std::string("Welcome to Graph Editor! :)\n") +
		std::string("1. Create and add graph with file data.\n") +
		std::string("2. Add vertex to graph.\n") +
		std::string("3. Add edge to graph.\n") +
		std::string("4. Delete vertex from graph.\n") +
		std::string("5. Delete edge from graph.\n") +
		std::string("6. Show current graph.\n") +
		std::string("7. Upload graph.\n") +
		std::string("8. Create graph with console.\n") +
		std::string("9. Show all graphs.\n") +
		std::string("10. Change current graph.\n") +
		std::string("11. Delete graph from graphs list.\n") +
		std::string("12. Change menu.\n") +
		std::string("Enter number of command.\n");

	std::string tasksMenu = std::string("1. IA. Degree oriented graph's vertex.\n") +
		std::string("2. IA_2, Neighbours over 1 vertex.\n") +
		std::string("3. IB, Symmetric difference.\n") +
		std::string("4. II5, Trees and forests.\n") +
		std::string("5. II6, Distances by edges.\n") +
		std::string("6. III7, MST by Prim's algorithm.\n") +
		std::string("7. IVA, Dijkstra.\n") +
		std::string("8. IVB, Graph radius.\n") +
		std::string("9. IVC, Floyd.\n") +
		std::string("10. V, Max Flow.\n") +
		std::string("11. Change menu.\n") +
		std::string("Enter number of command.\n");

	void generalMenuWork();
	void tasksMenuWork();

	void createGraph();
	void addVertex();
	void addEdge();
	void deleteVertex();
	void deleteEdge();
	void showGraph();
	void showCurrentGraph();
	void Upload();
	void consoleInput();
	void showAllGraphs();
	void changeCurrentGraph();
	void deleteGraphFromGraphList();
	void changeMenu();
	void IA();
	void IA_2();
	void IB();
	void II5();
	void II6();
	void III7();
	void IVA();
	void IVB();
	void IVC();
	void V();
};

