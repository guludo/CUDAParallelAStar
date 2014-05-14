/**
 * Serial implementation for Puzzle with dimension N.
 */
 
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "../serial/AStarSerial.h"

#define MAZE_AT(x,y) maze.m[(y)*maze.sizeX + x]

#define M_FREE	0
#define M_WALL	1
#define M_START	2
#define M_GOAL	3
#define M_PATH	4
#define M_POINT	5

typedef struct {
	int x;
	int y;
	int from;
	bool * points;
} State;

typedef struct {
	int sizeX;
	int sizeY;
	int * points;
	int pointsLength;
	int * m;
} Maze;

Maze maze;

State * start;

State * goal;

FILE * mazeFile;

//
//  command line option processing
//
int find_option( int argc, char **argv, const char *option )
{
	for( int i = 1; i < argc; i++ )
		if( strcmp( argv[i], option ) == 0 )
			return i;
	return -1;
}

int read_int( int argc, char **argv, const char *option, int default_value )
{
	int iplace = find_option( argc, argv, option );
	if( iplace >= 0 && iplace < argc-1 )
		return atoi( argv[iplace+1] );
	return default_value;
}

char *read_string( int argc, char **argv, const char *option, char *default_value )
{
	int iplace = find_option( argc, argv, option );
	if( iplace >= 0 && iplace < argc-1 )
		return argv[iplace+1];
	return default_value;
}

void loadMaze(char * mazeFilename){
	mazeFile = fopen(mazeFilename, "r");
	
	if(!mazeFile){
		fprintf(stderr, "Maze file not found.\n");
		exit(1);
	}
	
	fscanf(mazeFile, "%d,%d\n", &maze.sizeY, &maze.sizeX);
	printf("Maze dimensions: X,Y=%d,%d\n", maze.sizeY, maze.sizeX);
	int size = maze.sizeY * maze.sizeX;
	maze.m = (int *) malloc(sizeof(int) * size);
	char c;
	int i = 0;
	while((c = fgetc(mazeFile)) != EOF){
		switch(c){
			case '#':
				maze.m[i++] = M_WALL;
				break;
			case ' ':
				maze.m[i++] = M_FREE;
				break;
			case '@':
				maze.m[i] = M_START;
				start->y = i / maze.sizeX;
				start->x = i % maze.sizeX;
				printf("Start: %d, %d, %d\n", i, start->x, start->y);
				i++;
				break;
			case 'X':
				maze.m[i] = M_GOAL;
				goal->y = i / maze.sizeX;
				goal->x = i % maze.sizeX;
				printf("Goal: %d, %d, %d\n", i, goal->x, goal->y);
				i++;
				break;
			case '*':
				maze.m[i++] = M_POINT;
				maze.pointsLength++;
				break;
		}
		if(i >= size){
			break;
		}
	}
	int count = 0;
	maze.points = (int *) malloc(sizeof(int) * maze.pointsLength);
	for(i = 0; i<size && count<maze.pointsLength; i++){
		if(maze.m[i] == M_POINT){
			maze.points[count] = i;
			count++;
		}
	}
	start->points = (bool *) malloc(sizeof(bool) * maze.pointsLength);
	for(i = 0; i<maze.pointsLength; i++){
		start->points[i] = false;
	}
	
	fclose(mazeFile);
}

void printMaze(){
	for(int y = 0; y<maze.sizeY; y++){
		for(int x = 0; x<maze.sizeX; x++){
			switch(MAZE_AT(x,y)){
				case M_FREE:
					printf(" ");
					break;
				case M_WALL:
					printf("#");
					break;
				case M_START:
					printf("@");
					break;
				case M_GOAL:
					printf("X");
					break;
				case M_PATH:
					printf("'");
					break;
				case M_POINT:
					printf("*");
					break;
			}
		}
		printf("\n");
	}
}

void assignPathToMaze(AS_NodePointer * path){
	for(int i = 0; path[i]; i++){
		State * s = (State *) path[i]->state;
		if(MAZE_AT(s->x, s->y) == M_FREE) MAZE_AT(s->x, s->y) = M_PATH;
	}
}

bool areSameState(void * stateA, void * stateB){
	State * a = (State *) stateA;
	State * b = (State *) stateB;
	if(a->x == b->x && a->y == b->y && a->from == b->from){
		for(int i = 0; i<maze.pointsLength; i++){
			if(a->points[i] != b->points[i]){
				return false;
			}
		}
		return true;
	}
	return false;
}

bool isGoalState(void * state){
	State * s = (State *) state;
	if(s->x == goal->x && s->y == goal->y){
		for(int i = 0; i<maze.pointsLength; i++){
			if(!s->points[i]){
				return false;
			}
		}
		return true;
	}
	return false;
}

// double getHeuristic(void * state){
	// State * s = (State *) state;
	// double h = abs(s->x - goal->x) + abs(s->y - goal->y);
	// int count = 1;
	// for(int i = 0; i<maze.pointsLength; i++){
		// if(!s->points[i]){
			// int y = maze.points[i] / maze.sizeX;
			// int x = maze.points[i] % maze.sizeX;
			// h += abs(s->x - x) + abs(s->y - y);
			// count++;
		// }
	// }
	// return h/count;
// }

double getHeuristic(void * state){
	State * s = (State *) state;
	int count = 1;
	double h = -1;
	for(int i = 0; i<maze.pointsLength; i++){
		if(!s->points[i]){
			int y = maze.points[i] / maze.sizeX;
			int x = maze.points[i] % maze.sizeX;
			double hl = abs(s->x - x) + abs(s->y - y);
			if(h == -1){
				h = hl;
			}else{
				if(hl < h) h = hl;
			}
			count++;
		}
	}
	if(h == -1){
		h = abs(s->x - goal->x) + abs(s->y - goal->y);
	}
	return h;
}

AS_Node * createNode(AS_Node * oldNode, int x, int y){
	State * state = (State *) malloc(sizeof(State));
	State * oldState = (State *) oldNode->state;
	state->x = x;
	state->y = y;
	state->from = oldState->y*maze.sizeX + oldState->x;
	state->points = (bool *) malloc(sizeof(bool) * maze.pointsLength);
	memcpy(state->points, oldState->points, sizeof(bool) * maze.pointsLength);
	if(MAZE_AT(x,y) == M_POINT){
		int pointIndex = y*maze.sizeX + x;
		for(int i = 0; i<maze.pointsLength; i++){
			if(maze.points[i] == pointIndex){
				state->points[i] = true;
				break;
			}
		}
	}
	AS_Node * node = newASNode(getHeuristic(state), oldNode->cost + 1);
	node->state = state;
	return node;
}

AS_NodePointer * expandNode(AS_Node * node){
	State * s = (State *) node->state;
	int x = s->x;
	int y = s->y;
	AS_NodePointer * nodeList = (AS_NodePointer *) malloc(sizeof(AS_NodePointer)*5);
	
	int c = 0;
	if(x+1 < maze.sizeX && MAZE_AT(x+1,y) != M_WALL){
		nodeList[c++] = createNode(node, x+1, y);
	}
	if(x-1 >= 0 && MAZE_AT(x-1,y) != M_WALL){
		nodeList[c++] = createNode(node, x-1, y);
	}
	if(y-1 >= 0 && MAZE_AT(x,y-1) != M_WALL){
		nodeList[c++] = createNode(node, x, y-1);
	}
	if(y+1 < maze.sizeY && MAZE_AT(x,y+1) != M_WALL){
		nodeList[c++] = createNode(node, x, y+1);
	}
	nodeList[c] = NULL;
	
	return nodeList;
}

int main(int argc, char **argv){
	start = (State *) malloc(sizeof(State));
	start->from = -1;
	goal = (State *) malloc(sizeof(State));
	char defaultMaze[] = "maze1.txt";
	char * mazeFilename = read_string(argc, argv, "-m", defaultMaze);
	loadMaze(mazeFilename);
	
	AS_Node * startNode = newASNode(getHeuristic(start), 0);
	startNode->state = start;
	AS_Config config;
	AS_initConfig(&config);
	config.areSameStates = &areSameState;
	config.isGoalState = &isGoalState;
	config.expandNode = &expandNode;
	config.queueInitialCapacity = 20000;
	config.closedSetChunkSize = 20000;
	config.startNode = startNode;
	
	AS_NodePointer * path = AS_search(&config);
	
	printMaze();
	
	if(path){
		printf("Solution found.\n");
		assignPathToMaze(path);
		printMaze();
		AS_freePath(path);
	}else{
		printf("Solution not found\n.");
	}
	return 0;
}
