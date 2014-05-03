#ifndef __AS__

#define AS_CLOSEDSET_CHUNK_SIZE 1000
#define AS_QUEUE_INITIAL_CAPACITY 1000
#define AS_QUEUE_INCREASE_CAPACITY 500
#define AS_NODES_PER_CYCLE 8
#define AS_MAX_NODES_PER_EXPANSION 20


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
	double cost;
	double heuristic;
} NodeData;

typedef void (*ExpandStateFunction)(void * state, void * expansionStates, NodeData * nodesData, int * expansionLength);

typedef struct {
	AS_Node *	startNode;
	bool		(* areSameStates)(void * stateA, void * stateB);
	bool		(* isGoalState)(void * state);
	ExpandStateFunction	expandState;
	int			closedSetChunkSize;
	int			queueInitialCapacity;
	int			queueIncreaseCapacity;
	int			nodesPerCycle;
	int			maxNodesPerExpansion;
	size_t		stateSize;
	size_t		dataSize;
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
 * Makes the node expansion.
 * Output: expansionNodes - a NULL ended array with the nodes of the expansion.
 */
void AS_nodeCycle(AS_Node * node, AS_NodePointer * expansionNodes);
/**
 * Returns the NULL ended array of the path found to the goal.
 * And also clears those nodes in the tree that are not part of the path.
 * The parameter node is the goal leaf node in the tree.
 */
AS_NodePointer * AS_searchResult(AS_Node * node, AS_Config * config);

/**
 * Frees the path created by AS_searchResult.
 */
void AS_freePath(AS_NodePointer * path, AS_Config * config);
/**
 * Creates a new node and defines its default values.
 */
AS_Node * newASNode(void * state = NULL, double heuristic = 0, double cost = 1, AS_Node * parent = NULL);
/**
 * Initialize a node with default values.
 */
__host__ __device__ void ASNode_init(AS_Node * node, void * state = NULL, double heuristic = 0, double cost = 1, AS_Node * parent = NULL);
/**
 * Frees a node.
 */
void ASNode_free(AS_Node * node, AS_Config * config);
/**
 * Frees a tree. Nodes with status AS_STATUS_IN_PATH are not freed.
 */
void AS_freeTree(AS_Node * root, AS_Config * config);


#endif
