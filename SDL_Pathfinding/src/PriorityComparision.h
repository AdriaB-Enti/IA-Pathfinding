#pragma once
#include "SteeringBehavior.h"
#include "Node.h"

class PriorityComparision
{
public:
	bool operator()(const Node& a, const Node& b) {
		return a.priority < b.priority;
	}
};
