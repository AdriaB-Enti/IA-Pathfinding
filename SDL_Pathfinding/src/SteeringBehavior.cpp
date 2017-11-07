#include "SteeringBehavior.h"

using namespace std;

SteeringBehavior::SteeringBehavior()
{
}

SteeringBehavior::~SteeringBehavior()
{
}

Vector2D SteeringBehavior::Seek(Agent *agent, Vector2D target, float dtime)
{
	Vector2D desired_velocity = target - agent->position;
	desired_velocity.Normalize();
	desired_velocity *= agent->max_velocity;

	Vector2D steering_force = desired_velocity - agent->velocity;
	steering_force /= agent->max_velocity;
	steering_force = steering_force * agent->max_force;

	return steering_force;
}

Vector2D SteeringBehavior::Seek(Agent *agent, Agent *target, float dtime)
{
	return Seek(agent, target->position, dtime);
}

Vector2D SteeringBehavior::Arrive(Agent *agent, Vector2D target, int slow_radius, float dtime)
{
	float slow_factor = (target - agent->position).Length() / slow_radius;

	Vector2D desired_velocity = target - agent->position;
	desired_velocity.Normalize();
	if ((target - agent->position).Length() > slow_radius)
		desired_velocity *= agent->max_velocity;
	else
		desired_velocity *= agent->max_velocity * slow_factor;

	Vector2D steering_force = desired_velocity - agent->velocity;
	//steering_force /= agent->max_velocity;
	//steering_force = steering_force * agent->max_force;

	return steering_force;
}

Vector2D SteeringBehavior::Arrive(Agent *agent, Agent *target, int slow_radius, float dtime)
{
	return Arrive(agent, target->position, slow_radius, dtime);
}
std::vector<Vector2D> SteeringBehavior::BreadthFirstSearch(Graph graph, Vector2D firstPos, Vector2D goal) {
	vector<Vector2D> frontier;
	frontier.push_back(firstPos); //Posem la primera posicio
	Vector2D current;	
	vector<Vector2D> visited; //mentre no trobo com fer el camefrom

	while (!frontier.empty()) {
		current = frontier[0]; //agafem el primer de la frontera		
		
		for each (Connection c in graph.GetConnections(current)) // comprovem els seus veïns
		{
			if (visited.empty() || std::find(visited.begin(),visited.end(),c.getToNode()) != visited.end()) { //si no els haviem visitat els afegim a frontera
				visited.push_back(c.getToNode());
				frontier.push_back(c.getToNode());
				if (c.getToNode() == goal) //si hem trobat el goal sortim. No ho faig després de definir el current pq afegim els nodes al final del  frontier i potser trigo unes quantes iteracions a arrivar-hi.
					goto createpath;
			}
		}
		frontier.erase(frontier.begin()); //esborrem aquesta posicio pq ja l'hem comprovat
	}	
	
	createpath:
	//Retornem el cami
	vector<Vector2D> path;
	path.push_back(firstPos);	
	return path;
}