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
	map<Vector2D,Vector2D> came_from;	
	vector<Vector2D> path;		

	//Comprovem nodes fins al goal
	while (!frontier.empty()) {
		current = frontier[0]; //agafem el primer de la frontera		
		for each (Connection c in graph.GetConnections(current)) // comprovem els seus veïns
		{			
			if (came_from.find(c.getToNode()) == came_from.end()) { //si no els haviem visitat els afegim a frontera
				
				if (c.getToNode() == goal) {
					cout << "GOAL" << endl;
					goto createpath;
				}
				//cout << current.x << "," << current.y << " | " << c.getToNode().x << "," << c.getToNode().y << endl;
				pair<Vector2D, Vector2D> temp = make_pair(c.getToNode(), current);
				came_from.insert(temp);
				//cout << prova.second.x << " " << prova.second.y << endl;				
				frontier.push_back(c.getToNode());							
			}
			
		}		
		frontier.erase(frontier.begin()); //esborrem aquesta posicio pq ja l'hem comprovat		
	}	
	createpath:
	//Creem el camí	
	cout << "CHECKED CELLS : "<< came_from.size() << endl;	
	/*current = goal;
	path.insert(path.begin(), current);
	while (current != firstPos) {
		current = came_from[current];
		//path.insert(path.begin(), current);
	}*/
	return path;
}