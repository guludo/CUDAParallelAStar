#ifndef __QUEUE_H__
#define __QUEUE_H__

#include "AStarCUDA.h"

typedef struct{
	AS_NodePointer * list;
	int capacity;
	int index;
} Queue; /* The priority queue of leaf nodes to be used on A* search */
/**
 * Creates the priority queue.
 */
Queue * newQueue(int capacity);
/**
 * Inserts a node in the priority queue.
 * The lower the sum of the cumulated cost and heuristic value, the higher the priority.
 */
void Queue_insert(Queue * queue, AS_NodePointer node);
/**
 * Verifies if the queue is empty.
 */
inline bool Queue_isEmpty(Queue * queue);
/**
 * Removes the node with higher priority from the queue.
 */
AS_NodePointer Queue_remove(Queue * queue);
/**
 * Frees the queue.
 */
void Queue_free(Queue * queue);

#endif
