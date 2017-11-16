#pragma once
#include <iostream>
#include "Vector2D.h"


class Connection
{
public:
	Connection(Vector2D _fromNode, Vector2D _toNode, float cost);
	~Connection();

	float GetCost();
	Vector2D getFromNode();
	Vector2D getToNode();
	inline bool operator<(const Connection& rhs) const
	{
		if (cost < rhs.cost)
			return true;
	}
private:
	float cost;
	Vector2D fromNode;
	Vector2D toNode;

};

