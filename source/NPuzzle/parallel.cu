/**
 * Serial implementation for Puzzle with dimension N.
 */
 
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <sys/time.h>
#include "../parallel2/AStarCUDA.h"

//#define PATH_PRINT


int dimension;

State * start;

bool areSameState(void * stateA, void * stateB){
	State * a = (State *) stateA;
	State * b = (State *) stateB;
	int l = dimension * dimension;
	for(int i = 0; i<l; i++){
		if(a[i] != b[i]) return false;
	}
	return true;
}

bool isGoalState(void * state){
	State * s = (State *) state;
	int l = dimension * dimension;
	int i;
	for(i = 0; i<l-1; i++){
		if(s[i] != i+1) return false;
	}
	return s[i] == 0;
}


void printState(State * state){
	for(int i = 0; i<dimension; i++){
		for(int j = 0; j<dimension; j++){
			printf("%d ", state[i*dimension + j]);
		}
		printf("\n");
	}
}

void printPath(AS_NodePointer * path){
	for(int i = 0; path[i]; i++){
		printState((State *) path[i]->state);
		printf("\n");
	}
}

bool stateHasSolution(State * s){
	int sum = 0;
	int l = dimension * dimension;
	for(int i = 0; i<l; i++){
		if(s[i] == 0) continue;
		for(int j = i+1; j<l; j++){
			if(s[j] == 0) continue;
			if(s[j] < s[i]){
				sum += 1;
			}
		}
	}
	return sum % 2 == 0;
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
	dimension = read_int(argc, argv, "-d", 3);
	unsigned int seed = read_int(argc, argv, "-r", time(NULL) );
	const char * heuristicFunction = read_string(argc, argv, "-h", (char *)"manhattan");
	if (!strcmp (heuristicFunction, "hamming")) {
		heuristic = HAMMING;
		printf("Hamming heuristic\n");
	}
	else if (!strcmp (heuristicFunction, "manhattan")) {
		heuristic = MANHATTAN;
		printf("Manhattan heuristic\n");
	}
	srand (seed);
	
	start = (State *) malloc(sizeof(State) * (dimension * dimension + 3);

	int l = dimension * dimension;
	for(int i = 0; i<l; i++){
		start[i] = 0;
	}
	
	//Create a random state
	for(int i = 1; i<l; i++){
		int index = rand() % l;
		while(start[index] != 0) index = rand() % l;
		start[index] = i;
	}
	
	for (int i = 0; i < dimension * dimension; i++) {
		if (start[i] == 0) {
			start[dimension * dimension] = i/dimension;
			start[dimension * dimension+1] = i%dimension;
			break;
		}
	}

	start[dimension * dimension + 2] = -2;
	
	AS_Node * startNode = newASNode(getHeuristic(start));
	startNode->state = start;
	
	AS_Config config;
	AS_initConfig(&config);
	config.areSameStates = &areSameState;
	config.isGoalState = &isGoalState;
	config.expandNode = &expandNode;
	config.queueInitialCapacity = 20000;
	config.closedSetChunkSize = 20000;
	config.startNode = startNode;
	
	printf("Initial state:\n");
	printState(start);
	double simulation_time = read_timer( );
	AS_NodePointer * path = AS_search(&config);
	simulation_time = read_timer( ) - simulation_time;
	
	if(path){
		printf("Solution found.\n");
#ifdef PATH_PRINT
		printPath(path);
#endif
		AS_freePath(path);
	}else{
		printf("Solution not found\n.");
	}
	printf( "simulation time = %g seconds", simulation_time);
	return 0;
	
}