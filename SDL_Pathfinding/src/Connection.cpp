#pragma once
#include "Connection.h"

Connection::Connection(Vector2D _fromNode, Vector2D _toNode, float _cost)
{
	fromNode = _fromNode;
	toNode = _toNode;
	cost = _cost;
	
}

Connection::~Connection()
{
}

float Connection::GetCost()
{
	return cost;
}

Vector2D Connection::getFromNode()
{	
	return fromNode;
}

Vector2D Connection::getToNode()
{
	return toNode;
}
