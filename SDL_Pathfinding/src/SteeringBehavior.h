#pragma once
#include <vector>
#include <concurrent_priority_queue.h>
#include <map>
#include <queue>
#include "Agent.h"
#include "Vector2D.h"
#include "Graph.h"
#include <algorithm>
#include "Node.h"
#include "PriorityComparision.h"

class Agent;

class SteeringBehavior
{
public:
	SteeringBehavior();
	~SteeringBehavior();

	Vector2D Seek(Agent *agent, Vector2D target, float dtime);
	Vector2D Seek(Agent *agent, Agent *target, float dtime);
	Vector2D Arrive(Agent *agent, Vector2D target, int slow_radius, float dtime);
	Vector2D Arrive(Agent *agent, Agent *target, int slow_radius, float dtime);
	std::vector<Vector2D> BreadthFirstSearch(Graph graph, Vector2D firstPos, Vector2D goal);
	std::vector<Vector2D> SceneGreedyBFS(Graph graph, Vector2D firstPos, Vector2D goal);
	bool FindInMap(std::map<Vector2D, Vector2D> m, Vector2D objective);
	Vector2D ReturnMapValue(std::map<Vector2D, Vector2D> m, Vector2D objective);
	std::vector<Vector2D> ASearch(Graph graph, Vector2D firstPos, Vector2D goal);
	
};