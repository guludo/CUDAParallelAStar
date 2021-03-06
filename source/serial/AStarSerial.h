#ifndef __AS__

#define AS_CLOSEDSET_CHUNK_SIZE 1000
#define AS_QUEUE_INITIAL_CAPACITY 1000
#define AS_QUEUE_INCREASE_CAPACITY 500
#define AS_STATUS_IDLE 1
#define AS_STATUS_IN_PATH 2

typedef struct _AS_Node {
	struct _AS_Node * parent;
	void * state;
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
	bool		(* areSameStates)(void * stateA, void * stateB);
	bool		(* isGoalState)(void * state);
	AS_NodePointer *	(* expandNode)(AS_Node * node);
	int			closedSetChunkSize;
	int			queueInitialCapacity;
	int			queueIncreaseCapacity;
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
 * Frees the path created by AS_searchResult.
 */
void AS_freePath(AS_NodePointer * path);
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


#endif