#pragma once
#include "SteeringBehavior.h"
#include "Node.h"

class PriorityComparision
{
public:
	bool operator()(const Node& a, const Node& b) {
		if (a.priority < b.priority || a.priority == b.priority) //els nodes amb prioritats més properes a 0 són els que volem que estiguin primer a la cua
			return false;
		return true;
	}
};
