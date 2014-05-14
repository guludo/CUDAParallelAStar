/**
 * Serial implementation for Puzzle with dimension N.
 */
 
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "../openmp/AStarMP.h"

typedef int State;

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

double getHeuristic(void * state){
	State * s = (State *) state;
	int l = dimension * dimension;
	double h = 0;
	int i;
	for(i = 0; i<l-1; i++){
		if(s[i] != i+1) h++;
	}
	if(s[i] == 0) h++;
	return h;
}
// double getHeuristic(void * state){
	// State * s = (State *) state;
	// double h = 0;
	// int d = dimension;
	
	// for(int i = 0; i<d; i++){
		// for(int j = 0; j<d; j++){
			// int v = s[i*d + j] -1;
			// if(v >= 0){
				// int vi = v / d;
				// int vj = v % d;
				// h += abs(vi -i) + abs(vj -j);
			// }
		// }
	// }
	// return h;
// }

bool stateHasSolution(State * s){
	int sum = 0;
	int l = dimension * dimension;
	int blankRow;
	for(int i = 0; i<l; i++){
		if(s[i] == 0){
			blankRow = dimension -(i / dimension); //From bottom
			continue;
		}
		for(int j = i+1; j<l; j++){
			if(s[j] == 0) continue;
			if(s[j] < s[i]){
				sum += 1;
			}
		}
	}
	bool solvable;
	if(dimension % 2 == 0){
		if(blankRow % 2 == 0){
			solvable = sum % 2 != 0;
		}else{
			solvable = sum % 2 == 0;
		}
	}else{
		solvable = sum % 2 == 0;
	}
	if(solvable) printf("Has solution.\n");
	else printf("Does not have solution.\n");
	return solvable;
}

// void createNode(AS_Node * node, State * oldState, int swapIndexA, int swapIndexB){
	// State * state = (State *) malloc(sizeof(int)*dimension*dimension);
	// memcpy(state, oldState, sizeof(int)*dimension*dimension);
	// state[swapIndexA] = oldState[swapIndexB];
	// state[swapIndexB] = oldState[swapIndexA];
	// ASNode_init(node, getHeuristic(state));
	// node->state = state;
// }

AS_Node * createNode(State * oldState, int swapIndexA, int swapIndexB){
	State * state = (State *) malloc(sizeof(int)*dimension*dimension);
	memcpy(state, oldState, sizeof(int)*dimension*dimension);
	state[swapIndexA] = oldState[swapIndexB];
	state[swapIndexB] = oldState[swapIndexA];
	AS_Node * node = newASNode(getHeuristic(state));
	node->state = state;
	return node;
}

void expandNode(AS_Node * node, AS_NodePointer * nodeList, int processIndex){

	State * state = (State *) node->state;
	int l = dimension * dimension;
	/* Look for the empty tile position */
	int emtpyTileI, emtpyTileJ, emptyTileIndex; 
	for(int i = 0; i<l; i++){
		if(state[i] == 0){
			emptyTileIndex = i;
			emtpyTileI = i / dimension;
			emtpyTileJ = i % dimension;
			break;
		}
	}
	
	if(processIndex == 0 && emtpyTileI-1 >= 0){
		nodeList[0] = createNode(state, emptyTileIndex, emptyTileIndex-dimension);
	}
	
	if(processIndex == 1 && emtpyTileI+1 < dimension){
		nodeList[1] = createNode(state, emptyTileIndex, emptyTileIndex+dimension);
	}
	
	if(processIndex == 2 && emtpyTileJ-1 >= 0){
		nodeList[2] = createNode(state, emptyTileIndex, emptyTileIndex-1);
	}
	
	if(processIndex == 3 && emtpyTileJ+1 < dimension){
		nodeList[3] = createNode(state, emptyTileIndex, emptyTileIndex+1);
	}
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
	dimension = read_int(argc, argv, "-d", 3);
	int nodesPerCycle = read_int(argc, argv, "-c", 8);
	
	start = (State *) malloc(sizeof(State) * dimension * dimension);
	
	do{
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
	}while(!stateHasSolution(start));
	
	
	AS_Node * startNode = newASNode(getHeuristic(start));
	startNode->state = start;
	
	AS_Config config;
	AS_initConfig(&config);
	config.areSameStates = &areSameState;
	config.isGoalState = &isGoalState;
	config.expandNode = &expandNode;
	config.queueInitialCapacity = 20000;
	config.closedSetChunkSize = 30000;
	config.startNode = startNode;
	config.expansionProcesses = 4;
	config.maxNodesPerExpansion = 4;
	config.nodesPerCycle = nodesPerCycle;
	
	printf("Initial state:\n");
	printState(start);
	
	AS_NodePointer * path = AS_search(&config);
	
	if(path){
		printf("Solution found.\n");
		//printPath(path);
		AS_freePath(path);
	}else{
		printf("Solution not found\n.");
	}
	return 0;
	
}