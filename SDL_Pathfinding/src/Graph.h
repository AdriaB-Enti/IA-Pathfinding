#pragma once
#include "Connection.h"
#include "Vector2D.h"
#include <vector>

class Graph
{
public:
	Graph();
	~Graph();
	std::vector<Connection> GetConnections(Vector2D* fromNode);
private:
	
};
