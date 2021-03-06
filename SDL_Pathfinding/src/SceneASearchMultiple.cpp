#include "SceneASearchMultiple.h"

using namespace std;

SceneASearchMultiple::SceneASearchMultiple()
{
	draw_grid = false;

	num_cell_x = SRC_WIDTH / CELL_SIZE;
	num_cell_y = SRC_HEIGHT / CELL_SIZE;
	initMaze();
	loadTextures("../res/mazeCosts.png", "../res/coin.png");

	srand((unsigned int)time(NULL));

	Agent *agent = new Agent;
	agent->loadSpriteTexture("../res/soldier.png", 4);
	agents.push_back(agent);


	// set agent position coords to the center of a random cell
	Vector2D rand_cell(-1,-1);
	while (!isValidCell(rand_cell))
		rand_cell = Vector2D(1, 1);
		//rand_cell = Vector2D((float)(rand() % num_cell_x), (float)(rand() % num_cell_y));
		
	agents[0]->setPosition(cell2pix(rand_cell));

	// set the coin in a random cell (but at least 3 cells far from the agent)
	coinPosition = Vector2D(-1, -1);
	for (int i = 0; i < 2; i++) {		
		while ((!isValidCell(coinPosition)) || (Vector2D::Distance(coinPosition, rand_cell) < 3)) {
			coinPosition = Vector2D((float)(rand() % num_cell_x), (float)(rand() % num_cell_y));			
		}
		coins.push_back(cell2pix(coinPosition));
		coinPosition = Vector2D(-1, -1);
	}
	

	// PathFollowing next Target
	currentTarget = Vector2D(0, 0);
	currentTargetIndex = -1;

	//PRACTICA
	createGraph();
	//coins.push_back(cell2pix(Vector2D{5,1}));
	//coins.push_back(cell2pix(Vector2D{ 30,15 }));
	path.points = agents[0]->Behavior()->AMultipleSearch(graph, cell2pix(rand_cell), coins);
	
}

SceneASearchMultiple::~SceneASearchMultiple()
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

void SceneASearchMultiple::update(float dtime, SDL_Event *event)
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
			if (currentTargetIndex == path.points.size() - 1)
			{
				if (dist < 3)
				{
					path.points.clear();
					currentTargetIndex = -1;
					agents[0]->setVelocity(Vector2D(0,0));
					// if we have arrived to the coin, replace it ina random cell!
					
					coinPosition = Vector2D(-1, -1);
					coins.clear();
					for (int i = 0; i < 2; i++) {
						while ((!isValidCell(coinPosition)) || (Vector2D::Distance(coinPosition, agents[0]->getPosition()) < 3)) {
							coinPosition = Vector2D((float)(rand() % num_cell_x), (float)(rand() % num_cell_y));
						}
						coins.push_back(cell2pix(coinPosition));
						coinPosition = Vector2D(-1, -1);
					}
					//Creem cami un altre cop
					path.points = agents[0]->Behavior()->AMultipleSearch(graph, cell2pix(pix2cell(agents[0]->getPosition())), coins);
					
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
		agents[0]->update(Vector2D(0,0), dtime, event);
	}
}

void SceneASearchMultiple::draw()
{
	drawCosts();
	drawMaze();
	drawCoin();


	if (draw_grid)
	{
		SDL_SetRenderDrawColor(TheApp::Instance()->getRenderer(), 255, 255, 255, 127);
		for (int i = 0; i < SRC_WIDTH; i+=CELL_SIZE)
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
}

const char* SceneASearchMultiple::getTitle()
{
	return "SDL Steering Behaviors :: PathFinding1 Demo";
}

void SceneASearchMultiple::drawCosts()
{
	if (draw_grid)
	{
		for (unsigned int c = 0; c < costs.size(); c++)
		{
			switch (c)
			{
			case 0:
				SDL_SetRenderDrawColor(TheApp::Instance()->getRenderer(), 200, 200, 200, 255);		//Gris
				break;
			case 1:
				SDL_SetRenderDrawColor(TheApp::Instance()->getRenderer(), 0, 0, 110, 255);		//Blau
				break;
			case 2:
				SDL_SetRenderDrawColor(TheApp::Instance()->getRenderer(), 200, 200, 0, 255);	//Groc
				break;
			case 3:
				SDL_SetRenderDrawColor(TheApp::Instance()->getRenderer(), 200, 0, 0, 255);		//Vermell
				break;
			}
			SDL_RenderFillRect(TheApp::Instance()->getRenderer(), &costs[c]);
		}
	}
}

void SceneASearchMultiple::drawMaze()
{
	if (draw_grid)
	{

		SDL_SetRenderDrawColor(TheApp::Instance()->getRenderer(), 0, 0, 255, 255);
		for (unsigned int i = 0; i < maze_rects.size(); i++)
			SDL_RenderFillRect(TheApp::Instance()->getRenderer(), &maze_rects[i]);
	}
	else
	{
		SDL_RenderCopy(TheApp::Instance()->getRenderer(), background_texture, NULL, NULL );
	}
}

void SceneASearchMultiple::drawCoin()
{
	
	for each (Vector2D c in coins)
	{
		Vector2D coin_coords = c;		
		int offset = CELL_SIZE / 2;
		SDL_Rect dstrect = { (int)coin_coords.x - offset, (int)coin_coords.y - offset, CELL_SIZE, CELL_SIZE };
		SDL_RenderCopy(TheApp::Instance()->getRenderer(), coin_texture, NULL, &dstrect);
	}
	
}

void SceneASearchMultiple::initMaze()
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

	// Poso els rectangles dels costos
	SDL_Rect cost = { 0, 0, 640, 352 };		//super-esq:	1
	costs.push_back(cost);
	cost = { 640, 0, 640, 352 };			//super-dret:	3
	costs.push_back(cost);
	cost = { 0, 352, 640, 416 };			//inf-esq:		5
	costs.push_back(cost);
	cost = { 640, 352, 640, 416 };			//inf-dret:		20
	costs.push_back(cost);

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
			Vector2D cell_center ((float)(i*CELL_SIZE + offset), (float)(j*CELL_SIZE + offset));
			for (unsigned int b = 0; b < maze_rects.size(); b++)
			{
				if (Vector2DUtils::IsInsideRect(cell_center, (float)maze_rects[b].x, (float)maze_rects[b].y, (float)maze_rects[b].w, (float)maze_rects[b].h))
				{
					terrain[i][j] = 0;
				    break;
				}  
			}
			//si no hi ha un mur posem el cost que calgui
			if (terrain[i][j] != 0)
			{
				for (unsigned int c = 0; c < costs.size(); c++)
				{
					if (Vector2DUtils::IsInsideRect(cell_center, (float)costs[c].x, (float)costs[c].y, (float)costs[c].w, (float)costs[c].h))
					{
						switch (c)
						{
						case 0:
							terrain[i][j] = 1;
							break;
						case 1:
							terrain[i][j] = 3;
							break;
						case 2:
							terrain[i][j] = 5;
							break;
						case 3:
							terrain[i][j] = 20;
							break;
						}
						break;		//sortim del bucle
					}
				}
			}
		}
	}
}

bool SceneASearchMultiple::loadTextures(char* filename_bg, char* filename_coin)
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

Vector2D SceneASearchMultiple::cell2pix(Vector2D cell)
{
	int offset = CELL_SIZE / 2;
	return Vector2D(cell.x*CELL_SIZE + offset, cell.y*CELL_SIZE + offset);
}

Vector2D SceneASearchMultiple::pix2cell(Vector2D pix)
{
	return Vector2D((float)((int)pix.x/CELL_SIZE), (float)((int)pix.y / CELL_SIZE));
}

bool SceneASearchMultiple::isValidCell(Vector2D cell)
{
	if ((cell.x < 0) || (cell.y < 0) || (cell.x >= terrain.size()) || (cell.y >= terrain[0].size()) )
		return false;
	return !(terrain[(unsigned int)cell.x][(unsigned int)cell.y] == 0);
}

void SceneASearchMultiple::createGraph() {
		
	for (int i = 0; i < num_cell_x; i++) {
		for (int j = 0; j < num_cell_y; j++) {
			
			if (terrain[i][j] != 0) { //si no estem en un mur
				Vector2D fromcell(i, j);
				Vector2D toCell;

				toCell.x = i; toCell.y = j + 1;
				if (isValidCell(toCell) && terrain[i][j + 1] != 0) { // si no ens hem sortit del grid ni estem en un mur
					graph.AddConnection(cell2pix(fromcell), cell2pix(toCell), terrain[i][j]);
				}
				toCell.x = i; toCell.y = j - 1;
				if (isValidCell(toCell) && terrain[i][j - 1] != 0) {
					graph.AddConnection(cell2pix(fromcell), cell2pix(toCell), terrain[i][j]);
				}
				toCell.x = i + 1; toCell.y = j;
				if (isValidCell(toCell) && terrain[i + 1][j] != 0) {
					graph.AddConnection(cell2pix(fromcell), cell2pix(toCell), terrain[i][j]);
				}
				toCell.x = i - 1; toCell.y = j;
				if (isValidCell(toCell) && terrain[i - 1][j] != 0) {
					graph.AddConnection(cell2pix(fromcell), cell2pix(toCell), terrain[i][j]);
				}
			}			
		}
	}

	//Perqu� els bordes tinguin conexi�:
	for (int yOffset = 0; yOffset < 3; yOffset++)
	{
		graph.AddConnection(cell2pix(Vector2D(39, 10 + yOffset)), cell2pix(Vector2D(0, 10 + yOffset)), terrain[0][10 + yOffset]);
		graph.AddConnection(cell2pix(Vector2D(0, 10 + yOffset)), cell2pix(Vector2D(39, 10 + yOffset)), terrain[39][10 + yOffset]);
	}
}
void SceneASearchMultiple::teleportIfBridge() {
	
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