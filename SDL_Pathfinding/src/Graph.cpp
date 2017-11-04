#pragma once
#include "Graph.h"


Graph::Graph()
{
}

Graph::~Graph()
{
}

std::vector<Connection> Graph::GetConnections(Vector2D fromNode)
{
	std::vector<Connection> neighbours;

	for each (Connection c in connections)
	{
		if (c.getFromNode() == fromNode) {
			neighbours.push_back(c);
		}
	}
	return neighbours;
}
void Graph::AddConnection(Vector2D from, Vector2D to, float cost){
	Connection temp(from, to, cost);
	connections.push_back(temp);
}


