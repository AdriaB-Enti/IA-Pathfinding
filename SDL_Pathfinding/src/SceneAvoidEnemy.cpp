#include "SceneAvoidEnemy.h"

using namespace std;

SceneAvoidEnemy::SceneAvoidEnemy()
{
	draw_grid = false;

	num_cell_x = SRC_WIDTH / CELL_SIZE;
	num_cell_y = SRC_HEIGHT / CELL_SIZE;
	initMaze();
	loadTextures("../res/maze.png", "../res/coin.png");

	srand((unsigned int)time(NULL));

	Agent *agent = new Agent;
	agent->loadSpriteTexture("../res/soldier.png", 4);
	agents.push_back(agent);
	agent = new Agent;
	agent->loadSpriteTexture("../res/zombie1.png", 8);
	agents.push_back(agent);


	// set agent position coords to the center of a random cell
	Vector2D rand_cell(-1, -1);
	//while (!isValidCell(rand_cell))
		//rand_cell = Vector2D((float)(rand() % num_cell_x), (float)(rand() % num_cell_y));
	rand_cell = Vector2D{ 1,1 };
	agents[0]->setPosition(cell2pix(rand_cell));

	//la pos del enemy
	Vector2D enemyPos = cell2pix(Vector2D{ 12,8 });
	agents[1]->setPosition(enemyPos);

	// set the coin in a random cell (but at least 3 cells far from the agent)
	coinPosition = Vector2D(-1, -1);
	while ((!isValidCell(coinPosition)) || (Vector2D::Distance(coinPosition, rand_cell) < 3) || coinPosition == enemyPos)
		coinPosition = Vector2D((float)(rand() % num_cell_x), (float)(rand() % num_cell_y));

	
	enemyTarget = cell2pix(Vector2D{10,1});
	
	// PathFollowing next Target
	currentTarget = Vector2D(0, 0);
	currentTargetIndex = -1;

	//PRACTICA
	createGraph();
	
	//Primer cridem el ASearch normal i anirem modificant el path en l'update.
	coinPosition = Vector2D{ 15,1 };
	path.points = agents[0]->Behavior()->ASearch(graph, cell2pix(rand_cell), cell2pix(coinPosition));
	//enemyPath.points = agents[1]->Behavior()->ASearch(graph, enemyPos, cell2pix(enemyTarget));
}

SceneAvoidEnemy::~SceneAvoidEnemy()
{
	if (background_texture)
		SDL_DestroyTexture(background_texture);
	if (coin_texture)
		SDL_DestroyTexture(coin_texture);

	for (int i = 0; i < (int)agents.size(); i++)
	{
		delete agents[i];
	}
}

void SceneAvoidEnemy::update(float dtime, SDL_Event *event)
{

	/* Keyboard & Mouse events */
	switch (event->type) {
	case SDL_KEYDOWN:
		if (event->key.keysym.scancode == SDL_SCANCODE_SPACE)
			draw_grid = !draw_grid;
		break;
	default:
		break;
	}

	if ((currentTargetIndex == -1) && (path.points.size()>0))
		currentTargetIndex = 0;

	if (currentTargetIndex >= 0)
	{
		float dist = Vector2D::Distance(agents[0]->getPosition(), path.points[currentTargetIndex]);
		if (dist < path.ARRIVAL_DISTANCE)
		{
			//Intentem esquivar enemic
			path.points = agents[0]->Behavior()->AvoidEnemy(graph, cell2pix(pix2cell(agents[1]->getPosition())), 10, path.points, currentTargetIndex);

			if (currentTargetIndex == path.points.size() - 1)
			{
				if (dist < 3)
				{
					path.points.clear();
					currentTargetIndex = -1;
					agents[0]->setVelocity(Vector2D(0, 0));
					// if we have arrived to the coin, replace it ina random cell!
					if (pix2cell(agents[0]->getPosition()) == coinPosition)
					{
						coinPosition = Vector2D(-1, -1);
						while ((!isValidCell(coinPosition)) || (Vector2D::Distance(coinPosition, pix2cell(agents[0]->getPosition()))<3))
							coinPosition = Vector2D((float)(rand() % num_cell_x), (float)(rand() % num_cell_y));
						//Creem cami un altre cop
						path.points = agents[0]->Behavior()->ASearch(graph, cell2pix(pix2cell(agents[0]->getPosition())), cell2pix(coinPosition));
					}
				}
				else
				{
					Vector2D steering_force = agents[0]->Behavior()->Arrive(agents[0], currentTarget, path.ARRIVAL_DISTANCE, dtime);
					agents[0]->update(steering_force, dtime, event);
				}
				return;
			}
			currentTargetIndex++;			
		}

		currentTarget = path.points[currentTargetIndex];		
		teleportIfBridge(); //Si estem als bordes teleportem a l'altre costat

		Vector2D steering_force = agents[0]->Behavior()->Seek(agents[0], currentTarget, dtime);
		agents[0]->update(steering_force, dtime, event);		
	}
	else
	{
		agents[0]->update(Vector2D(0, 0), dtime, event);
	}

	//ENEMY
	/*if ((enemyTargetIndex == -1) && (enemyPath.points.size()>0))
		enemyTargetIndex = 0;

	if (enemyTargetIndex >= 0)
	{
		float dist = Vector2D::Distance(agents[1]->getPosition(), enemyPath.points[enemyTargetIndex]);
		if (dist < enemyPath.ARRIVAL_DISTANCE)
		{			

			if (enemyTargetIndex == enemyPath.points.size() - 1)
			{
				if (dist < 3)
				{
					enemyPath.points.clear();
					enemyTargetIndex = -1;
					agents[1]->setVelocity(Vector2D(0, 0));
					// if we have arrived to the coin, replace it ina random cell!
					if (pix2cell(agents[1]->getPosition()) == enemyTarget)
					{
						enemyTarget = Vector2D(-1, -1);
						while ((!isValidCell(enemyTarget)) || (Vector2D::Distance(enemyTarget, pix2cell(agents[1]->getPosition()))<3))
							enemyTarget = Vector2D((float)(rand() % num_cell_x), (float)(rand() % num_cell_y));
						//Creem cami un altre cop
						enemyPath.points = agents[1]->Behavior()->BreadthFirstSearch(graph, cell2pix(pix2cell(agents[1]->getPosition())), cell2pix(enemyTarget));
					}
				}
				else
				{
					Vector2D steering_force = agents[1]->Behavior()->Arrive(agents[1], currentEnemyTarget, enemyPath.ARRIVAL_DISTANCE, dtime);
					agents[1]->update(steering_force, dtime, event);
				}
				return;
			}
			enemyTargetIndex++;
		}

		currentEnemyTarget = enemyPath.points[enemyTargetIndex];
		teleportIfBridge(); //Si estem als bordes teleportem a l'altre costat
				
		Vector2D enemySf = agents[1]->Behavior()->Seek(agents[1], currentEnemyTarget, dtime);
		agents[1]->update(enemySf, dtime, event);
	}
	else
	{
		agents[1]->update(Vector2D(0, 0), dtime, event);
	}*/
	
	Vector2D enemySf = agents[1]->Behavior()->Seek(agents[1], enemyTarget, dtime);
	agents[1]->update(enemySf, dtime, event);
	float dist = Vector2D::Distance(agents[1]->getPosition(), enemyTarget);
	if (dist < 3)
	{
		enemyTarget = Vector2D((float)(rand() % num_cell_x), 200);		
		/*while (Vector2D::Distance(coinPosition, enemyTarget)) {
			enemyTarget = Vector2D((float)(rand() % num_cell_x), (float)(rand() % num_cell_y));			
		}*/
		enemyTarget = cell2pix(enemyTarget);
	}

}

void SceneAvoidEnemy::draw()
{
	drawMaze();
	drawCoin();


	if (draw_grid)
	{
		SDL_SetRenderDrawColor(TheApp::Instance()->getRenderer(), 255, 255, 255, 127);
		for (int i = 0; i < SRC_WIDTH; i += CELL_SIZE)
		{
			SDL_RenderDrawLine(TheApp::Instance()->getRenderer(), i, 0, i, SRC_HEIGHT);
		}
		for (int j = 0; j < SRC_HEIGHT; j = j += CELL_SIZE)
		{
			SDL_RenderDrawLine(TheApp::Instance()->getRenderer(), 0, j, SRC_WIDTH, j);
		}
	}

	for (int i = 0; i < (int)path.points.size(); i++)
	{
		draw_circle(TheApp::Instance()->getRenderer(), (int)(path.points[i].x), (int)(path.points[i].y), 15, 255, 255, 0, 255);
		if (i > 0 && abs(path.points[i - 1].x - path.points[i].x) < 100) //si la linia no �s massa llarga
			SDL_RenderDrawLine(TheApp::Instance()->getRenderer(), (int)(path.points[i - 1].x), (int)(path.points[i - 1].y), (int)(path.points[i].x), (int)(path.points[i].y));
	}

	draw_circle(TheApp::Instance()->getRenderer(), (int)currentTarget.x, (int)currentTarget.y, 15, 255, 0, 0, 255);

	agents[0]->draw();
	agents[1]->draw();
}

const char* SceneAvoidEnemy::getTitle()
{
	return "SDL Steering Behaviors :: PathFinding1 Demo";
}

void SceneAvoidEnemy::drawMaze()
{
	if (draw_grid)
	{

		SDL_SetRenderDrawColor(TheApp::Instance()->getRenderer(), 0, 0, 255, 255);
		for (unsigned int i = 0; i < maze_rects.size(); i++)
			SDL_RenderFillRect(TheApp::Instance()->getRenderer(), &maze_rects[i]);
	}
	else
	{
		SDL_RenderCopy(TheApp::Instance()->getRenderer(), background_texture, NULL, NULL);
	}
}

void SceneAvoidEnemy::drawCoin()
{
	Vector2D coin_coords = cell2pix(coinPosition);
	int offset = CELL_SIZE / 2;
	SDL_Rect dstrect = { (int)coin_coords.x - offset, (int)coin_coords.y - offset, CELL_SIZE, CELL_SIZE };
	SDL_RenderCopy(TheApp::Instance()->getRenderer(), coin_texture, NULL, &dstrect);
}

void SceneAvoidEnemy::initMaze()
{

	// Initialize a list of Rectagles describing the maze geometry (useful for collision avoidance)
	SDL_Rect rect = { 0, 0, 1280, 32 };
	maze_rects.push_back(rect);
	rect = { 608, 32, 64, 32 };
	maze_rects.push_back(rect);
	rect = { 0, 736, 1280, 32 };
	maze_rects.push_back(rect);
	rect = { 608, 512, 64, 224 };
	maze_rects.push_back(rect);
	rect = { 0,32,32,288 };
	maze_rects.push_back(rect);
	rect = { 0,416,32,320 };
	maze_rects.push_back(rect);
	rect = { 1248,32,32,288 };
	maze_rects.push_back(rect);
	rect = { 1248,416,32,320 };
	maze_rects.push_back(rect);
	rect = { 128,128,64,32 };
	maze_rects.push_back(rect);
	rect = { 288,128,96,32 };
	maze_rects.push_back(rect);
	rect = { 480,128,64,32 };
	maze_rects.push_back(rect);
	rect = { 736,128,64,32 };
	maze_rects.push_back(rect);
	rect = { 896,128,96,32 };
	maze_rects.push_back(rect);
	rect = { 1088,128,64,32 };
	maze_rects.push_back(rect);
	rect = { 128,256,64,32 };
	maze_rects.push_back(rect);
	rect = { 288,256,96,32 };
	maze_rects.push_back(rect);
	rect = { 480, 256, 320, 32 };
	maze_rects.push_back(rect);
	rect = { 608, 224, 64, 32 };
	maze_rects.push_back(rect);
	rect = { 896,256,96,32 };
	maze_rects.push_back(rect);
	rect = { 1088,256,64,32 };
	maze_rects.push_back(rect);
	rect = { 128,384,32,256 };
	maze_rects.push_back(rect);
	rect = { 160,512,352,32 };
	maze_rects.push_back(rect);
	rect = { 1120,384,32,256 };
	maze_rects.push_back(rect);
	rect = { 768,512,352,32 };
	maze_rects.push_back(rect);
	rect = { 256,640,32,96 };
	maze_rects.push_back(rect);
	rect = { 992,640,32,96 };
	maze_rects.push_back(rect);
	rect = { 384,544,32,96 };
	maze_rects.push_back(rect);
	rect = { 480,704,32,32 };
	maze_rects.push_back(rect);
	rect = { 768,704,32,32 };
	maze_rects.push_back(rect);
	rect = { 864,544,32,96 };
	maze_rects.push_back(rect);
	rect = { 320,288,32,128 };
	maze_rects.push_back(rect);
	rect = { 352,384,224,32 };
	maze_rects.push_back(rect);
	rect = { 704,384,224,32 };
	maze_rects.push_back(rect);
	rect = { 928,288,32,128 };
	maze_rects.push_back(rect);

	// Initialize the terrain matrix (for each cell a zero value indicates it's a wall)

	// (1st) initialize all cells to 1 by default
	for (int i = 0; i < num_cell_x; i++)
	{
		vector<int> terrain_col(num_cell_y, 1);
		terrain.push_back(terrain_col);
	}
	// (2nd) set to zero all cells that belong to a wall
	int offset = CELL_SIZE / 2;
	for (int i = 0; i < num_cell_x; i++)
	{
		for (int j = 0; j < num_cell_y; j++)
		{
			Vector2D cell_center((float)(i*CELL_SIZE + offset), (float)(j*CELL_SIZE + offset));
			for (unsigned int b = 0; b < maze_rects.size(); b++)
			{
				if (Vector2DUtils::IsInsideRect(cell_center, (float)maze_rects[b].x, (float)maze_rects[b].y, (float)maze_rects[b].w, (float)maze_rects[b].h))
				{
					terrain[i][j] = 0;
					break;
				}
			}

		}
	}
}

bool SceneAvoidEnemy::loadTextures(char* filename_bg, char* filename_coin)
{
	SDL_Surface *image = IMG_Load(filename_bg);
	if (!image) {
		cout << "IMG_Load: " << IMG_GetError() << endl;
		return false;
	}
	background_texture = SDL_CreateTextureFromSurface(TheApp::Instance()->getRenderer(), image);

	if (image)
		SDL_FreeSurface(image);

	image = IMG_Load(filename_coin);
	if (!image) {
		cout << "IMG_Load: " << IMG_GetError() << endl;
		return false;
	}
	coin_texture = SDL_CreateTextureFromSurface(TheApp::Instance()->getRenderer(), image);

	if (image)
		SDL_FreeSurface(image);

	return true;
}

Vector2D SceneAvoidEnemy::cell2pix(Vector2D cell)
{
	int offset = CELL_SIZE / 2;
	return Vector2D(cell.x*CELL_SIZE + offset, cell.y*CELL_SIZE + offset);
}

Vector2D SceneAvoidEnemy::pix2cell(Vector2D pix)
{
	return Vector2D((float)((int)pix.x / CELL_SIZE), (float)((int)pix.y / CELL_SIZE));
}

bool SceneAvoidEnemy::isValidCell(Vector2D cell)
{
	if ((cell.x < 0) || (cell.y < 0) || (cell.x >= terrain.size()) || (cell.y >= terrain[0].size()))
		return false;
	return !(terrain[(unsigned int)cell.x][(unsigned int)cell.y] == 0);
}

void SceneAvoidEnemy::createGraph() {

	for (int i = 0; i < num_cell_x; i++) {
		for (int j = 0; j < num_cell_y; j++) {

			if (terrain[i][j] != 0) { //si no estem en un mur
				Vector2D fromcell(i, j);
				Vector2D toCell;

				toCell.x = i; toCell.y = j + 1;
				if (isValidCell(toCell) && terrain[i][j + 1] != 0) { // si no ens hem sortit del grid ni estem en un mur
					graph.AddConnection(cell2pix(fromcell), cell2pix(toCell), 1);
				}
				toCell.x = i; toCell.y = j - 1;
				if (isValidCell(toCell) && terrain[i][j - 1] != 0) {
					graph.AddConnection(cell2pix(fromcell), cell2pix(toCell), 1);
				}
				toCell.x = i + 1; toCell.y = j;
				if (isValidCell(toCell) && terrain[i + 1][j] != 0) {
					graph.AddConnection(cell2pix(fromcell), cell2pix(toCell), 1);
				}
				toCell.x = i - 1; toCell.y = j;
				if (isValidCell(toCell) && terrain[i - 1][j] != 0) {
					graph.AddConnection(cell2pix(fromcell), cell2pix(toCell), 1);
				}
			}
		}
	}

	//Agefim conexions als bordes
	Vector2D fromCell(39, 10);
	Vector2D toCell(0, 10);
	graph.AddConnection(cell2pix(fromCell), cell2pix(toCell), 1);
	fromCell = { 39,11 };
	toCell = { 0,11 };
	graph.AddConnection(cell2pix(fromCell), cell2pix(toCell), 1);
	fromCell = { 39,12 };
	toCell = { 0,12 };
	graph.AddConnection(cell2pix(fromCell), cell2pix(toCell), 1);
}
void SceneAvoidEnemy::teleportIfBridge() {

	if (currentTarget == cell2pix(Vector2D{ 0,10 })) {
		if (path.points[currentTargetIndex - 1] == cell2pix(Vector2D{ 39,10 }))
			agents[0]->setPosition(cell2pix(Vector2D{ 0,10 }));
	}
	else if (currentTarget == cell2pix(Vector2D{ 0,11 })) {
		if (path.points[currentTargetIndex - 1] == cell2pix(Vector2D{ 39,11 }))
			agents[0]->setPosition(cell2pix(Vector2D{ 0,11 }));
	}
	else if (currentTarget == cell2pix(Vector2D{ 0,12 })) {
		if (path.points[currentTargetIndex - 1] == cell2pix(Vector2D{ 39,12 }))
			agents[0]->setPosition(cell2pix(Vector2D{ 0,12 }));
	}
	else if (currentTarget == cell2pix(Vector2D{ 39,10 })) {
		if (path.points[currentTargetIndex - 1] == cell2pix(Vector2D{ 0,10 }))
			agents[0]->setPosition(cell2pix(Vector2D{ 39,10 }));
	}
	else if (currentTarget == cell2pix(Vector2D{ 39,11 })) {
		if (path.points[currentTargetIndex - 1] == cell2pix(Vector2D{ 0,11 }))
			agents[0]->setPosition(cell2pix(Vector2D{ 39,11 }));
	}
	else if (currentTarget == cell2pix(Vector2D{ 39,12 })) {
		if (path.points[currentTargetIndex - 1] == cell2pix(Vector2D{ 0,12 }))
			agents[0]->setPosition(cell2pix(Vector2D{ 39,12 }));
	}
}