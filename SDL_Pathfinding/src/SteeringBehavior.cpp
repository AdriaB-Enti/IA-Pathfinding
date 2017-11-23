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
	//Creem el cam?		
	current = goal;
	path.insert(path.begin(), current);
	while (current != firstPos) {
		current = ReturnMapValue(came_from, current);
		path.insert(path.begin(), current);
	}
	cout << "NODES EXPLORATS: " << totalExploredNodes << ", NODES VISITATS : " << visitedNodes << endl;
	
	return path;
}
std::vector<Vector2D> SteeringBehavior::Dijkstra(Graph graph, Vector2D firstPos, Vector2D goal)
{
	priority_queue<Node, vector<Node>, PriorityComparision> frontier;
	map<Vector2D, Vector2D> came_from;
	map<Vector2D, float> cost_so_far;
	Node current = { firstPos, 0 };
	
	int totalExploredNodes = 0;											//Per poder fer les comparacions amb els altres algoritmes
	int visitedNodes = 0;

	frontier.push(current);												//Posem la primera posicio a la frontera
	came_from.emplace(make_pair(firstPos, NULL));						//Afegim el node als visitats
	cost_so_far.emplace(make_pair(firstPos, 0));						//Posem a 0 el cost de la primera posici?
	//Comprovem nodes fins al goal
	while (!frontier.empty()) {
		current = frontier.top();										//Agafem el primer de la frontera		
		frontier.pop();													//Esborrem el node

		for each (Connection c in graph.GetConnections(current.position))
		{
			totalExploredNodes++;
			float newCost = ReturnMapValue(cost_so_far, current.position) + c.GetCost();
			if (!FindInMap(came_from, c.getToNode()) || newCost < ReturnMapValue(cost_so_far, c.getToNode())) { //si no els haviem visitat els afegim a frontera
				visitedNodes++;
				cost_so_far.emplace(c.getToNode(), newCost);				//afegim el cost fins a current
				Node next = { c.getToNode(), newCost };
				frontier.push(next);										//afegim el next amb la seva prioritat
				came_from.emplace(make_pair(c.getToNode(), current.position));

				//Si em trobat la destinació parem
				if (c.getToNode() == goal) {
					cout << "GOAL" << endl;
					goto createpathDijkstra;
				}
			}
		}
	}

createpathDijkstra:
	//Creem el path
	vector<Vector2D> path;
	Vector2D currentPos = goal;
	path.insert(path.begin(), currentPos);
	while (currentPos != firstPos) {
		currentPos = ReturnMapValue(came_from, currentPos);
		path.insert(path.begin(), currentPos);
	}

	cout << "NODES EXPLORATS: " << totalExploredNodes << ", NODES VISITATS : " << visitedNodes << endl;
	return path;
}

std::vector<Vector2D> SteeringBehavior::SceneGreedyBFS(Graph graph, Vector2D firstPos, Vector2D goal) {

	vector<Vector2D> path;

	priority_queue<Node, vector<Node>, PriorityComparision> frontier;
	float priority;
	map<Vector2D, Vector2D> came_from;

	struct Node firstNode = { firstPos, 1 };
	frontier.emplace(firstNode);
	struct Node current;

	int totalExploredNodes = 0;
	int visitedNodes = 0;

	//Iterem la frontera
	while (!frontier.empty()) {
		current = frontier.top();
		frontier.pop();//el borrem ara perque si després afegim un amb més prioritat no borrarem el que toca
					   //cout << "CURRENT : " << current.position.x << "," << current.position.y << endl;
		for each (Connection c in graph.GetConnections(current.position)) // comprovem els seus veïns
		{
			totalExploredNodes++;
			
			if (!FindInMap(came_from, c.getToNode()) && c.getToNode() != firstPos) { //si no els haviem visitat els afegim a frontera
																					 //cout << "POS ACCEPTADA : " << c.getToNode().x << "," << c.getToNode().y << endl;
				visitedNodes++;
				//afegim nou cost
				
				/*std::map<Vector2D, float>::iterator it = cost_so_far.begin();
				// Iterate over the map using Iterator till end.
				while (it != cost_so_far.end())
				{
				cout << it->first.x << "," << it->first.y << " COST -> " << it->second << endl;
				it++;
				}	*/

				//afegim a la frontera amb prioritat de cost + heuristica
				priority = ManhattanDistance(c.getToNode(), goal);
				Node next = { c.getToNode(), priority };
				frontier.push(next);

				//afegim al came_from per recuperar despres el path
				came_from[c.getToNode()] = current.position;

				if (c.getToNode() == goal) {
					cout << "GOAL" << endl;
					goto createpath;
				}
			}

		}
		/*cout << "FRONTERA" << endl;
		priority_queue<Node, vector<Node>, PriorityComparision> prova = frontier;
		while (!prova.empty()) {
		cout << prova.top().position.x << "," << prova.top().position.y << " | PRIORITY : " << prova.top().priority << endl;
		prova.pop();
		}*/

	}

createpath:
	//Creem el cam?
	Vector2D posInPath;

	posInPath = goal;
	path.push_back(posInPath);
	while (posInPath != firstPos) {
		posInPath = ReturnMapValue(came_from, posInPath);
		path.insert(path.begin(), posInPath);
	}

	cout << "NODES EXPLORATS: " << totalExploredNodes << ", NODES VISITATS : " << visitedNodes << endl;
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
bool SteeringBehavior::FindInMap(std::map<Vector2D, float> m, Vector2D objective) {
	// Creem iterador
	std::map<Vector2D, float>::iterator it = m.begin();

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
float SteeringBehavior::ReturnMapValue(std::map<Vector2D, float> m, Vector2D objective) {
	std::map<Vector2D, float>::iterator it = m.begin();

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
	
	vector<Vector2D> path;
	
	priority_queue<Node, vector<Node>,PriorityComparision> frontier;
	float priority;
	map<Vector2D, Vector2D> came_from;
	
	struct Node firstNode = { firstPos, 1 };	
	frontier.emplace(firstNode);
	struct Node current;
	
	map<Vector2D, float> cost_so_far;
	float new_cost;
	pair<Vector2D, float> temp = make_pair(firstPos, 0);
	cost_so_far.emplace(temp);

	int totalExploredNodes = 0;
	int visitedNodes = 0;	
	
	//Iterem la frontera
	while (!frontier.empty()) {
		current = frontier.top();
		frontier.pop();//el borrem ara perque si després afegim un amb més prioritat no borrarem el que toca
		//cout << "CURRENT : " << current.position.x << "," << current.position.y << endl;
		for each (Connection c in graph.GetConnections(current.position)) // comprovem els seus veïns
		{
			totalExploredNodes++;
			new_cost = ReturnMapValue(cost_so_far,current.position) + c.GetCost();
			
			if (!FindInMap(cost_so_far, c.getToNode()) || new_cost < ReturnMapValue(cost_so_far, c.getToNode())) { //si no haviem calculat el cost o és més petit
				//cout << "POS ACCEPTADA : " << c.getToNode().x << "," << c.getToNode().y << endl;
				visitedNodes++;
				//afegim nou cost
				pair<Vector2D, int> tempCost = make_pair(c.getToNode(), new_cost);
				cost_so_far.insert(tempCost);

				/*std::map<Vector2D, float>::iterator it = cost_so_far.begin();
				// Iterate over the map using Iterator till end.
				while (it != cost_so_far.end())
				{
					cout << it->first.x << "," << it->first.y << " COST -> " << it->second << endl;
					it++;
				}	*/			

				//afegim a la frontera amb prioritat de cost + heuristica
				priority = new_cost + ManhattanDistance(c.getToNode(), goal);
				Node next = { c.getToNode(), priority };
				frontier.push(next);
				
				//afegim al came_from per recuperar despres el path
				came_from[c.getToNode()] = current.position;				

				if (c.getToNode() == goal) {
					cout << "GOAL" << endl;
					goto createpath;					
				}
			}
			
		}
		/*cout << "FRONTERA" << endl;
		priority_queue<Node, vector<Node>, PriorityComparision> prova = frontier;
		while (!prova.empty()) {
			cout << prova.top().position.x << "," << prova.top().position.y << " | PRIORITY : " << prova.top().priority << endl;
			prova.pop();
		}*/

	}
	
	createpath:
	//Creem el cam?
	Vector2D posInPath;
	
	posInPath = goal;
	path.push_back(posInPath);
	while (posInPath != firstPos) {
		posInPath = ReturnMapValue(came_from, posInPath);
		path.insert(path.begin(), posInPath);
	}
	
	cout << "NODES EXPLORATS: " << totalExploredNodes << ", NODES VISITATS : " << visitedNodes << endl;	
	return path;
}

std::vector<Vector2D> SteeringBehavior::AMultipleSearch(Graph graph, Vector2D firstPos, vector<Vector2D> goals) {

	vector<Vector2D> path;
	vector<Vector2D> objectives = goals;
	Vector2D lastGoal = firstPos;
	vector<Vector2D> miniPath;
	Vector2D posInPath;
	
	priority_queue<Node, vector<Node>, PriorityComparision> frontier;
	float priority;
	map<Vector2D, Vector2D> came_from;

	struct Node firstNode = { firstPos, 0 };
	frontier.emplace(firstNode);
	struct Node current;

	map<Vector2D, float> cost_so_far;
	float new_cost;
	pair<Vector2D, float> temp = make_pair(firstPos, 0);
	cost_so_far.emplace(temp);

	int totalExploredNodes = 0;
	int visitedNodes = 0;

	//Iterem la frontera
	while (!frontier.empty()) {
		current = frontier.top();
		frontier.pop();//el borrem ara perque si després afegim un amb més prioritat no borrarem el que toca
					  
		for each (Connection c in graph.GetConnections(current.position)) // comprovem els seus veïns
		{
			totalExploredNodes++;
			new_cost = ReturnMapValue(cost_so_far, current.position) + c.GetCost();

			if (!FindInMap(cost_so_far, c.getToNode()) || new_cost < ReturnMapValue(cost_so_far, c.getToNode())) {
				//path.push_back(c.getToNode());
				visitedNodes++;
				//afegim nou cost
				pair<Vector2D, float> tempCost = make_pair(c.getToNode(), new_cost);
				cost_so_far.insert(tempCost);

				//calculem la millor heuristica
				float bestHeuristic = ManhattanDistance(c.getToNode(), objectives[0]);
				for each (Vector2D g in objectives)
				{
					float thisHeuristic = ManhattanDistance(c.getToNode(), g);
					if (thisHeuristic < bestHeuristic)
						bestHeuristic = thisHeuristic;
				}
				//afegim a la frontera amb prioritat de cost + heuristica
				priority = new_cost + bestHeuristic;
				Node next = { c.getToNode(), priority };
				frontier.push(next);

				//afegim al came_from per recuperar despres el path				
				pair<Vector2D, Vector2D> temp = make_pair(c.getToNode(), current.position);
				came_from.insert(temp);
								
				//comprovem si hem arrivat a algun goal
				for each (Vector2D g in objectives)
				{
					if (c.getToNode() == g) {
						if(objectives.size() == 1)
							goto createpath;
						else { // si no és l'ultim nod el borrem de l'array i netegem la cua perque comenci a partir del objectiu actual
							
							//Fiquem en el path
							posInPath = g;
							miniPath.push_back(posInPath);
							while (posInPath != lastGoal) {
								posInPath = ReturnMapValue(came_from, posInPath);
								miniPath.insert(miniPath.begin(), posInPath);
							}
							lastGoal = g;
							path.insert(path.end(), miniPath.begin(), miniPath.end());
							miniPath.clear();

							//netegem variables
							map<Vector2D, Vector2D> emptyMap;
							came_from = emptyMap;
							priority_queue<Node, vector<Node>, PriorityComparision> emptyPqueue;
							frontier = emptyPqueue;
							Node next = { c.getToNode(), 0 };
							frontier.push(next);
							map<Vector2D, float> emptyCostMap;
							cost_so_far = emptyCostMap;
							pair<Vector2D, float> newFirstCost = make_pair(c.getToNode(), 0);
							cost_so_far.emplace(newFirstCost);


							vector<Vector2D>::iterator position = std::find(objectives.begin(), objectives.end(), g);
							if (position != objectives.end())
								objectives.erase(position);
						}
							
					}
				}
			}

		}
	}

createpath:	
	//cout << came_from.size() << endl;
	//Creem el camí final
	posInPath = objectives[0];
	miniPath.push_back(posInPath);
	while (posInPath != lastGoal) {
		posInPath = ReturnMapValue(came_from, posInPath);
		miniPath.insert(miniPath.begin(), posInPath);
	}	
	path.insert(path.end(), miniPath.begin(), miniPath.end());
	
	cout << "NODES EXPLORATS: " << totalExploredNodes << ", NODES VISITATS : " << visitedNodes << endl;
	return path;
}

float SteeringBehavior::ManhattanDistance(Vector2D start, Vector2D goal) {
			
	//Com que el cost minim entre nodes es 1 hem de passar aquestes distances a posicio en la grid per a que pugui coincidir
	Vector2D startCell = Vector2D((float)((int)start.x / CELL_SIZE), (float)((int)start.y / CELL_SIZE));
	Vector2D goalCell = Vector2D((float)((int)goal.x / CELL_SIZE), (float)((int)goal.y / CELL_SIZE));
	//cout << "START CELL : " << startCell.x << "," << startCell.y << ", END CELL: " << goalCell.x << "," << goalCell.y << endl;

	float dx = abs(startCell.x - goalCell.x);
	float dy = abs(startCell.y - goalCell.y);
	return dx + dy; //No multipliquem per res perque el cos minim entre dos nodes es 1
}

std::vector<Vector2D> SteeringBehavior::AvoidEnemy(Graph graph, Vector2D firstPos, Vector2D goal, Vector2D enemy) {
	
	vector<Vector2D> path;

	priority_queue<Node, vector<Node>, PriorityComparision> frontier;
	float priority;
	map<Vector2D, Vector2D> came_from;

	struct Node firstNode = { firstPos, 1 };
	frontier.emplace(firstNode);
	struct Node current;

	map<Vector2D, float> cost_so_far;
	float new_cost;
	pair<Vector2D, float> temp = make_pair(firstPos, 0);
	cost_so_far.emplace(temp);

	int totalExploredNodes = 0;
	int visitedNodes = 0;

	//Iterem la frontera
	while (!frontier.empty()) {

		current = frontier.top();
		frontier.pop();
		cout << "COSEN POS : " << current.position.x << "," << current.position.y << endl;
		for each (Connection c in graph.GetConnections(current.position)) // comprovem els seus veïns
		{
			totalExploredNodes++;
			new_cost = ReturnMapValue(cost_so_far, current.position) + c.GetCost();

			if (!FindInMap(cost_so_far, c.getToNode()) || new_cost < ReturnMapValue(cost_so_far, c.getToNode())) { 				

				cout << "CHOSEN NEIGHBOUR :" << c.getToNode().x << "," << c.getToNode().y << endl;

				visitedNodes++;

				//afegim nou cost
				pair<Vector2D, int> tempCost = make_pair(c.getToNode(), new_cost);
				cost_so_far.insert(tempCost);
								
				//afegim a la frontera amb prioritat de cost + heuristica
				priority = new_cost + ManhattanDistance(c.getToNode(), goal) - ManhattanDistance(c.getToNode(), enemy)*2;
				
				cout << "GOAL DIST: " << ManhattanDistance(c.getToNode(), goal) << " vs ENEMY DIST: " << ManhattanDistance(c.getToNode(), enemy) << " || " << "P: " << priority << endl;

				if (priority < 0)
					priority = 0;
				Node next = { c.getToNode(), priority };
				frontier.push(next);

				//afegim al came_from per recuperar despres el path
				came_from[c.getToNode()] = current.position;

				if (c.getToNode() == goal) {
					cout << "GOAL" << endl;
					goto createpath;
				}
			}		

			cout << "FRONTERA" << endl;
			priority_queue<Node, vector<Node>, PriorityComparision> prova = frontier;
			while (!prova.empty()) {
				cout << prova.top().position.x << "," << prova.top().position.y << " | PRIORITY : " << prova.top().priority << endl;
				prova.pop();
			}
		}
	}

createpath:
	//Creem el cam?
	Vector2D posInPath;

	posInPath = goal;
	path.push_back(posInPath);
	while (posInPath != firstPos) {
		posInPath = ReturnMapValue(came_from, posInPath);
		path.insert(path.begin(), posInPath);
	}

	cout << "NODES EXPLORATS: " << totalExploredNodes << ", NODES VISITATS : " << visitedNodes << endl;
	return path;

}
