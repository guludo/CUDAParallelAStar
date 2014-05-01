/**
 * Serial implementation for Horse Moves.
 */

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "../pre-parallel/AStarPP.h"

typedef struct{
	int x;
	int y;
} State;

int dimension;
State goal;
State * start;

bool areSameState(void * stateA, void * stateB){
	State * a = (State *) stateA;
	State * b = (State *) stateB;
	return a->x == b->x && a->y == b->y;
}

bool isGoalState(void * state){
	return areSameState(state, &goal);
}

double getHeuristic(State * state){
	int dx = abs(goal.x - state->x);
	int dy = abs(goal.y - state->y);
	double h = (dx + dy)/3.0;
	if((dx % 2 == 0) && ((dx/2)%2 == 0)){
		h *= 0.9;
	}
	if((dy % 2 == 0) && ((dy/2)%2 == 0)){
		h *= 0.9;
	}
	return h;
}

void createNode(AS_Node * node, int x, int y){
	State * state = (State *) malloc(sizeof(State));
	state->x = x;
	state->y = y;
	ASNode_init(node, getHeuristic(state));
	node->state = state;
}

void expandNode(AS_Node * node, AS_Node * expansionNodes, int * expansionLength){
	State * state = (State *) node->state;
	int x = state->x;
	int y = state->y;
	int d = dimension;
	
	int count = 0;
	
	if(x -2 >= 0) {
		if(y - 2 >= 0){
			createNode(expansionNodes + count, x-2, y-1);
			count++;
			createNode(expansionNodes + count, x-1, y-2);
			count++;
		}else if(y - 1 >= 0){
			createNode(expansionNodes + count, x-2, y-1);
			count++;
		}
		if(y + 2 < d){
			createNode(expansionNodes + count, x-1, y+2);
			count++;
			createNode(expansionNodes + count, x-2, y+1);
			count++;
		}else if(y + 1 < d){
			createNode(expansionNodes + count, x-2, y+1);
			count++;
		}
	}else if(x -1 >= 0){
		if(y - 2 >= 0){
			createNode(expansionNodes + count, x-1, y-2);
			count++;
		}
		if(y + 2 < d){
			createNode(expansionNodes + count, x-1, y+2);
			count++;
		}
	}
	
	if(x + 2 < d){
		if(y - 2 >= 0){
			createNode(expansionNodes + count, x+2, y-1);
			count++;
			createNode(expansionNodes + count, x+1, y-2);
			count++;
		}else if(y - 1 >= 0){
			createNode(expansionNodes + count, x+2, y-1);
			count++;
		}
		if(y + 2 < d){
			createNode(expansionNodes + count, x+2, y+1);
			count++;
			createNode(expansionNodes + count, x+1, y+2);
			count++;
		}else if(y + 1 < d){
			createNode(expansionNodes + count, x+2, y+1);
			count++;
		}
	}else if(x + 1 < d){
		if(y - 2 >= 0){
			createNode(expansionNodes + count, x+1, y-2);
			count++;
		}
		if(y + 2 < d){
			createNode(expansionNodes + count, x+1, y+2);
			count++;
		}
	}
	
	*expansionLength = count;	
}

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

int main(int argc, char **argv){
	dimension = read_int(argc, argv, "-d", 12);
	start = (State *) malloc(sizeof(State));
	start->x = 0;
	start->y = 0;
	
	goal.x = dimension-1;
	goal.y = dimension-1;
	
	AS_Config config;
	AS_initConfig(&config);
	config.areSameStates = &areSameState;
	config.isGoalState = &isGoalState;
	config.expandNode = &expandNode;
	config.queueInitialCapacity = 20000;
	config.closedSetChunkSize = 20000;
	
	AS_Node * startNode = newASNode(getHeuristic(start));
	startNode->state = start;
	config.startNode = startNode;
	
	AS_NodePointer * path = AS_search(&config);
	
	if(path){
		State * s = (State *) path[0]->state;
		printf("Horse moves to go from position (%d,%d) to position (%d, %d):\n(%d, %d)", start->x, start->y, goal.x, goal.y, s->x, s->y);
		for(int i = 1; path[i]; i++){
			s = (State *) path[i]->state;
			printf(",(%d,%d)", s->x, s->y);
		}
		AS_freePath(path);
	}else{
		printf("Path not found from (%d,%d) to position (%d, %d):\n", start->x, start->y, goal.x, goal.y);
	}
	return 0;
}