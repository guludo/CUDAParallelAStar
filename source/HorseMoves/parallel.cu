/**
 * Serial implementation for Horse Moves.
 */

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <sys/time.h>
#include "../parallel/AStarSerial.h"

bool areSameState(void * stateA, void * stateB){
	State * a = (State *) stateA;
	State * b = (State *) stateB;
	return a->x == b->x && a->y == b->y;
}

bool areSameCost(AS_Node * a, AS_Node * b) {
	return a->cost + a->heuristic == b->cost + b->heuristic;
}

bool isGoalState(void * state){
	return areSameState(state, &goal);
}

AS_Node * createNode(int x, int y){
	State * state = (State *) malloc(sizeof(State));
	state->x = x;
	state->y = y;
	AS_Node * node = newASNode(getHeuristic(state));
	node->state = state;
	node->cur.x = x;
	node->cur.y = y;
	return node;
}

AS_NodePointer * expandNode(AS_Node * node){
	State * state = (State *) node->state;
	int x = state->x;
	int y = state->y;
	int d = dimension;
	
	AS_NodePointer * nodeList = (AS_NodePointer *) malloc(sizeof(AS_NodePointer)*9);
	int count = 0;
	
	if(x -2 >= 0) {
		if(y - 2 >= 0){
			nodeList[count++] = createNode(x-2, y-1);
			nodeList[count++] = createNode(x-1, y-2);
		}else if(y - 1 >= 0){
			nodeList[count++] = createNode(x-2, y-1);				
		}
		if(y + 2 < d){
			nodeList[count++] = createNode(x-2, y+1);
			nodeList[count++] = createNode(x-1, y+2);
		}else if(y + 1 < d){
			nodeList[count++] = createNode(x-2, y+1);
		}
	}else if(x -1 >= 0){
		if(y - 2 >= 0){
			nodeList[count++] = createNode(x-1, y-2);
		}
		if(y + 2 < d){
			nodeList[count++] = createNode(x-1, y+2);
		}
	}
	
	if(x + 2 < d){
		if(y - 2 >= 0){
			nodeList[count++] = createNode(x+2, y-1);
			nodeList[count++] = createNode(x+1, y-2);
		}else if(y - 1 >= 0){
			nodeList[count++] = createNode(x+2, y-1);				
		}
		if(y + 2 < d){
			nodeList[count++] = createNode(x+2, y+1);
			nodeList[count++] = createNode(x+1, y+2);
		}else if(y + 1 < d){
			nodeList[count++] = createNode(x+2, y+1);
		}
	}else if(x + 1 < d){
		if(y - 2 >= 0){
			nodeList[count++] = createNode(x+1, y-2);
		}
		if(y + 2 < d){
			nodeList[count++] = createNode(x+1, y+2);
		}
	}
	
	if(count < 8){
		nodeList = (AS_NodePointer *) realloc(nodeList, sizeof(AS_NodePointer)*(count+1));
	}
	nodeList[count] = NULL;

	return nodeList;
	
	
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

double read_timer( )
{
    static bool initialized = false;
    static struct timeval start;
    struct timeval end;
    if( !initialized )
    {
        gettimeofday( &start, NULL );
        initialized = true;
    }
    gettimeofday( &end, NULL );
    return (end.tv_sec - start.tv_sec) + 1.0e-6 * (end.tv_usec - start.tv_usec);
}

int main(int argc, char **argv){
	dimension = read_int(argc, argv, "-d", 12);
	start.x = 0;
	start.y = 0;
	goal.x = dimension-1;
	goal.y = dimension-1;
	
	AS_Config config;
	AS_initConfig(&config);
	config.areSameState = &areSameState;
	config.isGoalState = &isGoalState;
	config.areSameCost = &areSameCost;
	config.expandNode = &expandNode;
	
	AS_Node * startNode = newASNode(getHeuristic(&start));
	startNode->state = &start;
	config.startNode = startNode;	
	double simulation_time = read_timer( );
	AS_NodePointer * path = AS_search(&config);
	simulation_time = read_timer( ) - simulation_time;
	if(path){
		State * s = (State *) path[0]->state;
		printf("Horse moves to go from position (%d,%d) to position (%d, %d):\n(%d, %d)", start.x, start.y, goal.x, goal.y, s->x, s->y);
		for(int i = 1; path[i]; i++){
			s = (State *) path[i]->state;
			printf(",(%d,%d)", s->x, s->y);
		}
	}else{
		printf("Path not found from (%d,%d) to position (%d, %d):\n", start.x, start.y, goal.x, goal.y);
	}
	printf( "simulation time = %g seconds", simulation_time);

	// cleaning the memory
	cleanPath(path);
	cleanMem();
	return 0;
}

