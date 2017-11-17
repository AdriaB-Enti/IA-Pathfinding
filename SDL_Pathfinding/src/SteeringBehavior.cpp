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

	int totalExploredNodes = 0;
	int visitedNodes = 0;
	
	//Comprovem nodes fins al goal
	while (!frontier.empty()) {
		current = frontier[0]; //agafem el primer de la frontera		
		
		for each (Connection c in graph.GetConnections(current)) // comprovem els seus veïns
		{
			totalExploredNodes++;
			if (!FindInMap(came_from, c.getToNode()) && c.getToNode() != firstPos) { //si no els haviem visitat els afegim a frontera
					
				visitedNodes++;
				
				//Afegim al mapa i a la frontera
				pair<Vector2D, Vector2D> temp = make_pair(c.getToNode(), current);
				came_from.emplace(temp);
				frontier.push_back(c.getToNode());

				//DEBUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUGS
				//cout << "POS QUE S'HAURIA D'AFEGIR " << c.getToNode().x << "," << c.getToNode().y << endl;

				/*std::map<Vector2D, Vector2D>::iterator it = came_from.begin();
				// Iterate over the map using Iterator till end.
				while (it != came_from.end())
				{
					cout << it->first.x << "," << it->first.y << " VE DE -> " << it->second.x << "," << it->second.y << endl;
					it++;
				}*/
				
				//Sortim si hem trobat goal
				if (c.getToNode() == goal) {					
					goto createpath;
				}
			}
			
		}
		frontier.erase(frontier.begin()); //esborrem aquesta posicio pq ja l'hem comprovat		
	}	
	
	createpath:
	//Creem el camí			
	current = goal;
	path.insert(path.begin(), current);
	while (current != firstPos) {
		current = ReturnMapValue(came_from, current);
		path.insert(path.begin(), current);
	}
	cout << "NODES EXPLORATS: " << totalExploredNodes << ", NODES VISITATS : " << visitedNodes << endl;
	
	return path;
}
std::vector<Vector2D> SteeringBehavior::Dijkstra(Graph graph, Connection firstPos, Vector2D goal)
{

	std::priority_queue<Connection> frontier;
	frontier.push(firstPos); //Posem la primera posicio
	Vector2D current;
	Connection currentCon = Connection(Vector2D(), Vector2D(), 0);
	map<Vector2D, Vector2D> came_from;
	map<Vector2D, float> cost_so_far;
	vector<Vector2D> path;
	cout << "GOAL " << goal.x << " " << goal.y << endl;

	//Comprovem nodes fins al goal
	while (!frontier.empty()) {
		currentCon = frontier.top(); //agafem el primer de la frontera		
		current = currentCon.getToNode();

		for each (Connection c in graph.GetConnections(current))
		{
			if (!FindInMap(came_from, c.getToNode()) && c.getToNode() != firstPos.getToNode()) { //si no els haviem visitat els afegim a frontera

																								 //cout << "POS QUE S'HAURIA D'AFEGIR " << c.getToNode().x << "," << c.getToNode().y << endl;

																								 //Afegim al mapa i a la frontera
				pair<Vector2D, Vector2D> temp = make_pair(c.getToNode(), current);
				came_from.emplace(temp);
				frontier.push(c);

				//DEBUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUGS
				/*std::map<Vector2D, Vector2D>::iterator it = came_from.begin();
				// Iterate over the map using Iterator till end.
				while (it != came_from.end())
				{
				cout << it->first.x << "," << it->first.y << " VE DE -> " << it->second.x << "," << it->second.y << endl;
				it++;
				}*/
				///////////////////////////////////////////////////////////////////////////////////////////////////////////////

				//Sortim si hem trobat goal
				if (c.getToNode() == goal) {
					cout << "GOAL" << endl;
					goto createpathDijkstra;
				}
			}

		}

		frontier.pop(); //esborrem aquesta posicio pq ja l'hem comprovat	

	}


createpathDijkstra:
	//Creem el camí			
	current = goal;
	path.insert(path.begin(), current);
	while (current != firstPos.getToNode()) {
		current = ReturnMapValue(came_from, current);
		path.insert(path.begin(), current);
	}



	return path;
}

std::vector<Vector2D> SteeringBehavior::SceneGreedyBFS(Graph graph, Vector2D firstPos, Vector2D goal) {
	//template <typename T>
	//priority_queue<Vector2D, int, mycomparison> Frontier;
	
	


	vector<Vector2D> frontier;
	frontier.push_back(firstPos); //Posem la primera posicio
	Vector2D current;
	map<Vector2D, Vector2D> came_from;
	vector<Vector2D> path;
	/*
	//Comprovem nodes fins al goal
	while (!frontier.empty()) {

		current = frontier[0]; //agafem el primer de la frontera		

		for each (Connection c in graph.GetConnections(current)) // comprovem els seus veïns
		{
			if (!FindInMap(came_from, c.getToNode()) && c.getToNode() != firstPos) { //si no els haviem visitat els afegim a frontera

																					 //cout << "POS QUE S'HAURIA D'AFEGIR " << c.getToNode().x << "," << c.getToNode().y << endl;

																					 //Afegim al mapa i a la frontera
				pair<Vector2D, Vector2D> temp = make_pair(c.getToNode(), current);
				came_from.emplace(temp);
				frontier.push_back(c.getToNode());

				//DEBUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUGS
				/*std::map<Vector2D, Vector2D>::iterator it = came_from.begin();
				// Iterate over the map using Iterator till end.
				while (it != came_from.end())
				{
				cout << it->first.x << "," << it->first.y << " VE DE -> " << it->second.x << "," << it->second.y << endl;
				it++;
				}*/
				///////////////////////////////////////////////////////////////////////////////////////////////////////////////

				//Sortim si hem trobat goal
				/*if (c.getToNode() == goal) {
					//cout << "GOAL" << endl;
					goto createpath;
				}
			}

		}

		frontier.erase(frontier.begin()); //esborrem aquesta posicio pq ja l'hem comprovat		
	}

createpath:
	//Creem el camí			
	current = goal;
	path.insert(path.begin(), current);
	while (current != firstPos) {
		current = ReturnMapValue(came_from, current);
		path.insert(path.begin(), current);
	}

	*/
	return path;
}


bool SteeringBehavior::FindInMap(std::map<Vector2D, Vector2D> m, Vector2D objective) {
	// Creem iterador
	std::map<Vector2D, Vector2D>::iterator it = m.begin();

	// Iterate over the map using Iterator till end.
	while (it != m.end())
	{
		if (it->first == objective) {
			return true;
		}
		it++;
	}
	return false;
}

Vector2D SteeringBehavior::ReturnMapValue(std::map<Vector2D, Vector2D> m, Vector2D objective) {
	std::map<Vector2D, Vector2D>::iterator it = m.begin();

	// Iterate over the map using Iterator till end.
	while (it != m.end())
	{
		if (it->first == objective) {
			return it->second;
		}
		it++;
	}	
}

std::vector<Vector2D> SteeringBehavior::ASearch(Graph graph, Vector2D firstPos, Vector2D goal) {
	
	priority_queue<Node, vector<Node>,PriorityComparision> frontier;
	map<Vector2D, Vector2D> came_from;
	
	map<Vector2D, int> cost_so_far;
	float new_cost;

	struct Node firstNode = { firstPos, 1 };	
	frontier.emplace(firstNode);
	struct Node current;
	
	

	//Iterem la frontera
	/*while (!frontier.empty()) {
		current = frontier.top();
		frontier.pop();//el borrem ara perque si després afegim un amb més prioritat no borrarem el que toca
		for each (Connection c in graph.GetConnections(current.position)) // comprovem els seus veïns
		{
			//new_cost = cost_so_far
		}
	}
	*/
	vector<Vector2D> path;
	return path;
}

float SteeringBehavior::ManhattanDistance(Vector2D start, Vector2D goal) {
	
	//Com que el cost minim entre nodes es 1 hem de passar aquestes distances a posicio en la grid per a que pugui coincidir
	Vector2D startCell = Vector2D((float)((int)start.x / CELL_SIZE), (float)((int)start.x / CELL_SIZE));
	Vector2D goalCell = Vector2D((float)((int)goal.y / CELL_SIZE), (float)((int)goal.y / CELL_SIZE));
	
	float dx = abs(startCell.x - goalCell.x);
	float dy = abs(startCell.y - goalCell.y);
	return dx + dy; //No multipliquem per res perque el cos minim entre dos nodes es 1
}