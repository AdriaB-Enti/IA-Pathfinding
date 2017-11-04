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
	//std::cout << connections.size() << std::endl;
	for each (Connection c in connections)
	{		
		if (c.getFromNode().x == fromNode.x && c.getFromNode().y == fromNode.y) {			
			neighbours.push_back(c);
		}
	}
	return neighbours;
}
void Graph::AddConnection(Vector2D from, Vector2D to, float cost){
	Connection temp(from, to, cost);
	connections.push_back(temp);	
}


