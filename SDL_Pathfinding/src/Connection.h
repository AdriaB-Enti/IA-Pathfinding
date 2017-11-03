#pragma once
#include "Vector2D.h"


class Connection
{
public:
	Connection();
	~Connection();

	float GetCost();
	Vector2D getFromNode();
	Vector2D getToNode();

private:
	float cost;
	Vector2D fromNode;
	Vector2D toNode;

};