/**
 * Pre-parallel implementation of A* Search
 */

#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <float.h>
#include <string.h>
#include "AStarPP.h"
#include "ClosedSet.h"
#include "Queue.h"

/**
 * Constants used for node validation
 */
#define AS_V_IDLE		0 /* Node not visited yet */
#define AS_V_VALID		1 /* The node is valid */
#define AS_V_INVALID	2 /* The node is invalid */



using namespace std;


Queue * newQueue(int capacity, int increaseCapacity){
	Queue * queue = new Queue;
	queue->capacity = capacity+1;
	queue->increaseCapacity = increaseCapacity;
	queue->list = (AS_NodePointer *) malloc(sizeof(AS_NodePointer)*(queue->capacity));
	queue->index = 1;
	return queue;
}

void Queue_insert(Queue * queue, AS_NodePointer node){
	if(queue->index >= queue->capacity){
		queue->list = (AS_NodePointer *) realloc(queue->list, sizeof(AS_NodePointer)*(queue->capacity+AS_QUEUE_INCREASE_CAPACITY));
		if(!queue->list){
			fprintf(stderr, "Error on increasing queue capacity.\n");
			exit(1);			
		}
		queue->capacity += AS_QUEUE_INCREASE_CAPACITY;
	}
	int index = queue->index;
	queue->index++;
	double nodeCost = node->cost + node->heuristic;
	while(index/2 > 0){
		AS_NodePointer parent = queue->list[index/2];
		if(parent->cost + parent->heuristic > nodeCost){
			queue->list[index] = parent;
			index = index/2;
		}else{
			break;
		}
	}
	queue->list[index] = node;
}

inline bool Queue_isEmpty(Queue * queue){
	return queue->index == 1;
}

AS_NodePointer Queue_remove(Queue * queue){
	AS_NodePointer returnValue = queue->list[1];
	queue->index--;
	if(queue->index == 1) return returnValue;
	
	AS_NodePointer rightMost = queue->list[queue->index];
	double rightMostCost = rightMost->cost + rightMost->heuristic;
	int index = 2;
	while(index < queue->index){
		AS_NodePointer left = queue->list[index];
		AS_NodePointer right = index+1 < queue->index ? queue->list[index+1] : NULL;
		double leftCost = left->cost + left->heuristic;
		double rightCost = right == NULL ? DBL_MAX : right->cost + right->heuristic;
		if(rightMostCost > leftCost || rightMostCost > rightCost){
			int selectedIndex = index;
			if(right && rightCost < leftCost){
				selectedIndex++;
			}
			queue->list[index/2] = queue->list[selectedIndex];
			index = 2*selectedIndex;
		}else{
			break;
		}
	}
	queue->list[index/2] = rightMost;
	
	return returnValue;
}

void Queue_free(Queue * queue){
	free(queue->list);
	delete queue;
}

ClosedSet * newClosedSet(bool (* areSameStates)(void * stateA, void * stateB), int chunkSize = AS_CLOSEDSET_CHUNK_SIZE){
	ClosedSet * closedSet = new ClosedSet;
	closedSet->list = new ClosedSetList;
	closedSet->currentList = closedSet->list;
	closedSet->chunkSize = chunkSize;
	closedSet->length = 0;
	closedSet->areSameStates = areSameStates;
	
	closedSet->list->nodes = new AS_NodePointer[chunkSize];
	closedSet->list->next = NULL;
	return closedSet;
}

void ClosedSet_freeList(ClosedSetList * list){
	if(list->next) ClosedSet_freeList(list->next);
	delete [] list->nodes;
	delete list;
}

void ClosedSet_free(ClosedSet * closedSet){
	ClosedSet_freeList(closedSet->list);
	delete closedSet;
}

void ClosedSet_add(ClosedSet * closedSet, AS_NodePointer node){
	if(closedSet->length == 0){
		closedSet->currentList->nodes[0] = node;
	}else{
		int index = closedSet->length % closedSet->chunkSize;
		if(index == 0){
			closedSet->currentList->next = new ClosedSetList;
			closedSet->currentList = closedSet->currentList->next;
			closedSet->currentList->nodes = new AS_NodePointer[closedSet->chunkSize];
			closedSet->currentList->next = NULL;
		}
		closedSet->currentList->nodes[index] = node;
	}
	closedSet->length++;
}

bool ClosedSet_hasNode(ClosedSet * closedSet, AS_NodePointer node){
	ClosedSetList * list = closedSet->list;
	int chunkSize = closedSet->chunkSize;
	int length = closedSet->length;
	int count = 0;
	while(list){
		for(int i = 0; i<chunkSize && count<length; i++, count++){
			if(closedSet->areSameStates(node->state, list->nodes[i]->state)){
				return true;
			}
		}
		list = list->next;
	}
	return false;
}

void AS_initConfig(AS_Config * config){
	config->closedSetChunkSize = AS_CLOSEDSET_CHUNK_SIZE;
	config->queueInitialCapacity = AS_QUEUE_INITIAL_CAPACITY;
	config->queueIncreaseCapacity = AS_QUEUE_INCREASE_CAPACITY;
	config->nodesPerCycle = AS_NODES_PER_CYCLE;
	config->maxNodesPerExpansion = AS_MAX_NODES_PER_EXPANSION;
}

void AS_freeTree(AS_Node * root){
	if(root){
		for(int i = 0; i<root->childrenLength; i++){
			AS_freeTree(root->children[i]);
		}
		root->childrenLength = 0;
		if(root->status == AS_STATUS_IDLE){ //If it is not in the path
			ASNode_free(root);
		}
	}
}

AS_NodePointer * AS_searchResult(AS_Node * node){
	/* Count the number of nodes */
	AS_Node * n = node;
	int count = 0;
	while(n){
		count++;
		n = n->parent;
	}
	
	/* Creating array and filling it */
	AS_NodePointer * path = new AS_NodePointer[count+1];
	path[count] = NULL;
	count--;
	n = node;
	while(true){
		path[count] = n;
		n->status = AS_STATUS_IN_PATH;
		if(n->parent){
			n = n->parent;
			count--;
		}else{
			break;
		}
	}
	
	AS_freeTree(n);
	
	return path;
}

void AS_nodeCycle(AS_Config * config, AS_Node * node, AS_Node * expansionNodes, int * expansionLength){
	config->expandNode(node, expansionNodes, expansionLength);
}

AS_NodePointer * AS_search(AS_Config * config){
	AS_NodePointer * path = NULL;
	
	/* Allocate space for expansion */
	int expansionNodesLength = config->nodesPerCycle * (config->maxNodesPerExpansion);
	int sizeofExpansionNodes = sizeof(AS_Node) * expansionNodesLength;
	AS_Node * expansionNodes = (AS_Node *) malloc(sizeofExpansionNodes);
	int * expansionSizes = (int *) malloc(sizeof(int) * config->nodesPerCycle);
	AS_Node * hostExpansionNodes = (AS_Node *) malloc(sizeofExpansionNodes);
	int * hostExpansionSizes = (int *) malloc(sizeof(int) * config->nodesPerCycle);
	char * expansionValidity = (char *) malloc(sizeof(char) * expansionNodesLength);
	AS_NodePointer * nodesToExpand = (AS_NodePointer *) malloc(sizeof(AS_NodePointer) * config->nodesPerCycle);
	
	ClosedSet * closedSet = newClosedSet(config->areSameStates, config->closedSetChunkSize);
	Queue * queue = newQueue(config->queueInitialCapacity, config->queueIncreaseCapacity);
	
	Queue_insert(queue, config->startNode);
	
	int loopCount = 0;
	while(true){
		loopCount++;
		if(Queue_isEmpty(queue)){
			AS_freeTree(config->startNode);
			break;
		}
		
		nodesToExpand[0] = Queue_remove(queue);
		ClosedSet_add(closedSet, nodesToExpand[0]);
		
		/* If first element is the goal state */
		if(config->isGoalState(nodesToExpand[0]->state)){
			path = AS_searchResult(nodesToExpand[0]);
			break;
		}
		
		int nodeCount = 1;
		while(nodeCount < config->nodesPerCycle && !Queue_isEmpty(queue)){
			nodesToExpand[nodeCount] = Queue_remove(queue);
			ClosedSet_add(closedSet, nodesToExpand[nodeCount]);
			nodeCount++;
		}
		
		/* PSEUDO-PARALLELISM: Iterating blocks */
		for(int blockIndex = 0; blockIndex < nodeCount; blockIndex++){
			AS_nodeCycle(config, nodesToExpand[blockIndex], expansionNodes + config->maxNodesPerExpansion * blockIndex, expansionSizes + blockIndex);
		}
		/* Copy expansionNodes to host */
		memcpy(hostExpansionNodes, expansionNodes, sizeof(AS_Node)*nodeCount*config->maxNodesPerExpansion);
		memcpy(hostExpansionSizes, expansionSizes, sizeof(int)*nodeCount);
		
		
		/* 
		 * Mark which nodes in the expansion are valid, i.e., for repeated states,
		 * the state with the minimal cost is set as valid, the remainder as invalid.
		 * PS: Maybe we can make this parallel too.
		 */
		memset(expansionValidity, AS_V_IDLE, expansionNodesLength);
		/* li - localIndex, bi - blockIndex */
		for(int i = 0, bi=0, li=0; i<expansionNodesLength; ){
			if(li < hostExpansionSizes[bi] && expansionValidity[i] == AS_V_IDLE){
				expansionValidity[i] = AS_V_VALID;
				int validIndex = i;
				double validCost = hostExpansionNodes[i].cost + hostExpansionNodes[i].heuristic;
				/* lj - localIndex, bj - blockIndex */
				for(int bj = bi+1, lj = 0, j = bj*config->maxNodesPerExpansion; j<expansionNodesLength; ){
					bool goToNextBlock = false;
					if(lj < hostExpansionSizes[bj] && expansionValidity[j] == AS_V_IDLE){
						void * stateA = hostExpansionNodes[validIndex].state;
						void * stateB = hostExpansionNodes[j].state;
						if(config->areSameStates(stateA, stateB)){
							double costB = hostExpansionNodes[j].cost + hostExpansionNodes[j].heuristic;
							if(costB < validCost){
								expansionValidity[j] = AS_V_VALID;
								expansionValidity[validIndex] = AS_V_INVALID;
								validCost = costB;
								validIndex = j;
							}else{
								expansionValidity[j] = AS_V_INVALID;
							}
							goToNextBlock = true;
						}
					
					}
					if(lj + 1 < hostExpansionSizes[bj] && !goToNextBlock){
						j++;
						lj++;
					}else if(bj + 1 < nodeCount){
						bj++;
						j = bj * config->maxNodesPerExpansion;
						lj = 0;
					}else{
						break;
					}
				}
			}
			if(li + 1 < hostExpansionSizes[bi]){
				i++;
				li++;
			}else if(bi + 1 < nodeCount){
				bi++;
				i = bi * config->maxNodesPerExpansion;
				li = 0;
			}else{
				break;
			}
		}
		
		/* TODO: Add the valid expansion nodes to their parents */
		for(int i = 0; i<nodeCount; i++){
			AS_Node * children = hostExpansionNodes + config->maxNodesPerExpansion*i;
			int size = hostExpansionSizes[i];
			char * validity = expansionValidity + config->maxNodesPerExpansion*i;
			int j = 0;
			AS_Node * node = nodesToExpand[i];
			node->children = (AS_NodePointer *) malloc(sizeof(AS_NodePointer) * config->maxNodesPerExpansion);
			int c = 0; /* Counter for actual children */
			while(j<size){
				if(validity[j] && !ClosedSet_hasNode(closedSet, children + j)){
					/* Create a node and copy the child */
					children[j].parent = node;
					AS_Node * child = (AS_Node *) malloc(sizeof(AS_Node));
					memcpy(child, children + j, sizeof(AS_Node));
					node->children[c] = child;
					c++;
					Queue_insert(queue, child);
				}
				j++;
			}
			node->childrenLength = c;
			realloc(node->children, c*sizeof(AS_NodePointer));
		}
		/*
		AS_NodePointer * children = config->expandNode(node);
		int childrenLength = 0;
		int i = 0;
		while(children[i]){
			if(ClosedSet_hasNode(closedSet, children[i])){
				ASNode_free(children[i]);
				children[i] = NULL;
			}else{
				children[i]->parent = node;
				Queue_insert(queue, children[i]);
				childrenLength++;
			}
			i++;
		}
		node->childrenLength = childrenLength;
		if(childrenLength){
			node->children = new AS_NodePointer[childrenLength];
			int k = 0;
			for(int j = 0; j<i; j++){
				if(children[j]){
					node->children[k] = children[j];
					k++;
				}
			}
		}
		free(children);
		*/
	}
	
	printf("loop count = %d\n", loopCount);
	
	Queue_free(queue);
	ClosedSet_free(closedSet);
	
	free(expansionNodes);
	free(expansionSizes);
	free(hostExpansionNodes);
	free(hostExpansionSizes);
	free(expansionValidity);
	free(nodesToExpand);
	
	
	return path;
}

int deb = 0;
void AS_freePath(AS_NodePointer * path){
	for(int i = 0; path[i]; i++){
		ASNode_free(path[i]);
	}
	delete [] path;
}

AS_Node * newASNode(double heuristic, double cost, AS_Node * parent){
	AS_Node * node = (AS_Node *) malloc(sizeof(AS_Node));
	ASNode_init(node, heuristic, cost, parent);
	return node;
}

void ASNode_init(AS_Node * node, double heuristic, double cost, AS_Node * parent){
	node->data = NULL;
	node->state = NULL;
	node->parent = parent;
	node->heuristic = heuristic;
	node->cost = cost;
	node->status = AS_STATUS_IDLE;
	node->childrenLength = 0;
	node->children = NULL;
}

void ASNode_free(AS_Node * node){
	if(node->state){
		free(node->state);
		node->state = NULL;
	}
	if(node->data){
		free(node->data);
		node->data = NULL;
	}
	free(node->children);
	free(node);
}

void testQueue(){
	Queue * q = newQueue(1000, 100);
	
	printf("Testing with only one element:\n");
	printf("\tInserting...\n");
	Queue_insert(q, newASNode(1));
	printf("\tRemoving...\n");
	AS_Node * n = Queue_remove(q);
	printf("\tHeuristic: %f\n\n", n->heuristic);
	delete n;
	
	printf("Testing with 2 elements:\n");
	Queue_insert(q, newASNode(2));
	Queue_insert(q, newASNode(1));
	printf("\tExpected output: 1 2\n\tRemoving elements and printing values:\n");
	n = Queue_remove(q);
	printf("\t%f ", n->heuristic);
	delete n;
	n = Queue_remove(q);
	printf("%f\n\n ", n->heuristic);
	
	
	printf("Testing with 5 elements with random heuristics:\n\tInput order (heuristic values):");
	for(int i = 0; i<5; i++){
		int h = rand() % 100;
		printf(" %d", h);
		Queue_insert(q, newASNode(h));
	}
	printf("\n\tOutput:");
	for(int i = 0; i<5; i++){
		n = Queue_remove(q);
		printf(" %d", (int) n->heuristic);
		delete n;
	}
	printf("\n\tQueue is empty: %s", Queue_isEmpty(q) ? "YES" : "NO");
	
	Queue_free(q);
}

bool testClosedSet_isSameState(void * stateA, void * stateB){
	int a = *((int *) stateA);
	int b = *((int *) stateB);
	return a == b;
}
void testClosedSet(){
	ClosedSet * cs = newClosedSet(&testClosedSet_isSameState);
	AS_NodePointer n = newASNode();
	int * x = new int;
	*x = 1;
	n->state = x;
	
	printf("Adding only one element...\n");
	ClosedSet_add(cs, n);
	bool hasNode = ClosedSet_hasNode(cs, n);
	printf("\tHas node (expected YES): %s\n", hasNode ? "YES" : "NO");
	
	printf("Adding more 5 elements...\n");
	for(int i = 0; i<5; i++){
		AS_NodePointer n = newASNode();
		int * x = new int;
		*x = rand();
		n->state = x;
		ClosedSet_add(cs, n);
	}
	
	printf("Adding more CHUNK SIZE number of elements...\n");
	for(int i = 0; i<AS_CLOSEDSET_CHUNK_SIZE; i++){
		n = newASNode();
		x = new int;
		*x = rand();
		n->state = x;
		ClosedSet_add(cs, n);
	}
	printf("Verifying if the closed set has the last element added...\n");
	hasNode = ClosedSet_hasNode(cs, n);
	printf("\tHas node (expected YES): %s\n", hasNode ? "YES" : "NO");
	
	ClosedSet_free(cs);
}
