typedef struct _ClosedSetList{
	AS_NodePointer * nodes;
	struct _ClosedSetList * next;
} ClosedSetList; /* The list for the closed set */

typedef struct{
	ClosedSetList * list;
	ClosedSetList * currentList;
	int length;
	int chunkSize;
	bool (* areSameStates)(void * stateA, void * stateB);
} ClosedSet; /* The closed set to be used on A* search */
/**
 * Creates the closed set to be used in the A* search.
 * The parameter areSameStates is a function that must return true is two states are the same, and false otherwise.
 * This data structure allocates an initial amount of memory to store the nodes pointers, so that they can be in a contiguous region of memory.
 * This amount is defined by chunkSize, which has as default value AS_CLOSEDSET_CHUNK_SIZE.
 * When inserting a new node pointer, there's no space, a new chunk of memory for chunkSize node pointers is allocated.
 */
ClosedSet * newClosedSet(bool (* areSameStates)(void * stateA, void * stateB), int chunkSize);
/**
 * Recursively frees the closed set list.
 */
void ClosedSet_freeList(ClosedSetList * list);
/**
 * Frees the closed set.
 */
void ClosedSet_free(ClosedSet * closedSet);
/**
 * Adds a new node pointer to the closed set.
 */
void ClosedSet_add(ClosedSet * closedSet, AS_NodePointer node);
/**
 * Makes a linear search on the closed set to verify if it has a node pointer.
 */
bool ClosedSet_hasNode(ClosedSet * closedSet, AS_NodePointer node);
