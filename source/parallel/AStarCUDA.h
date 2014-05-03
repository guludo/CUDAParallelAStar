#ifndef __ASTARCUDA_H__
#define __ASTARCUDA_H__

#define AS_CLOSEDSET_CHUNK_SIZE 100
#define AS_QUEUE_INITIAL_CAPACITY 1000
#define AS_QUEUE_ADITIONAL_CAPACITY 500
#define AS_STATUS_IDLE 1
#define AS_STATUS_IN_PATH 2
#define NUM_BLOCKS 16
#define NUM_CHOICES 8


typedef struct{
	int x;
	int y;
} State;

extern State goal;
extern State start;
extern int dimension;

typedef struct _AS_Node {
	struct _AS_Node * parent;
	void * state;
	State cur;
	State prev;
	void * data;
	double heuristic;
	double cost;
	struct _AS_Node * * children;
	int childrenLength;
	int status;
} AS_Node;

typedef AS_Node * AS_NodePointer;

typedef struct {
	AS_Node *	startNode;
	bool		(* areSameState)(void * stateA, void * stateB);
	bool		(* isGoalState)(void * state);
	bool		(* areSameCost)(AS_Node * a, AS_Node * b);
	AS_NodePointer *	(* expandNode)(AS_Node * node);
	int			closedSetChunkSize;
	int			queueInitialCapacity;
} AS_Config;

/**
 * Initializes the configuration structure by setting the default values.
 */
void AS_initConfig(AS_Config * config);
/**
 * Performs the A* Search. The parameter config defines different parameters for the search.
 */
AS_NodePointer * AS_search(AS_Config * config);
/**
 * Returns the NULL ended array of the path found to the goal.
 * And also clears those nodes in the tree that are not part of the path.
 * The parameter node is the goal leaf node in the tree.
 */
AS_NodePointer * AS_searchResult(AS_Node * node);
/**
 * Creates a new node and defines its default values.
 */
AS_Node * newASNode(double heuristic = 0, double cost = 1, AS_Node * parent = NULL);
/**
 * Frees a node.
 */
void ASNode_free(AS_Node * node);
/**
 * Frees a tree. Nodes with status AS_STATUS_IN_PATH are not freed.
 */
void AS_freeTree(AS_Node * root);

/**
 * Cleans up dynamic memory
 */
void cleanMem();

/**
 * Cleans up path result
 */
void cleanPath(AS_NodePointer * path);

double getHeuristic(State * state); 

AS_Node * createNode(int x, int y);

#endif

