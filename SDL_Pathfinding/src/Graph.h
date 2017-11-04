#pragma once
#include "Connection.h"
#include "Vector2D.h"
#include <vector>

class Graph
{
public:
	Graph();
	~Graph();
	std::vector<Connection> GetConnections(Vector2D fromNode);
	void AddConnection(Vector2D from, Vector2D to, float cost);
private:
	std::vector<Connection> connections;
};
