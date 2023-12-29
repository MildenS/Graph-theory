#pragma once
#include<cstdint>
#include<string>

enum class GraphType : uint8_t
{
	Empty,
	WeightedOriented,
	UnweightedOriented,
	WeightedUndirected,
	UnweightedUndirected

};
enum class TreeType : uint8_t
{
	None,
	Tree,
	Forest,
	HasCycle,
	NotLinked
};

class GraphException
{
public:
	GraphException(std::string err) { error = err; }
	const std::string what() const { return error + '\n'; }
private:
	std::string error;
};


class GraphEdge
{
public:
	std::string firstVertex, secondVertex;
	int64_t dist;
	
	GraphEdge(std::string _firstVertex, std::string _secondVertex, int64_t _dist):
		firstVertex(_firstVertex), secondVertex(_secondVertex), dist(_dist) {}

	bool operator < (const GraphEdge& other) const
	{
		return (firstVertex < other.firstVertex) ||
			(firstVertex == other.firstVertex && secondVertex < other.secondVertex);
	}
};