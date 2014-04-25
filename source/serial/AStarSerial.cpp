#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <float.h>
#include "AStarSerial.h"
#include "ClosedSet.h"
#include "Queue.h"


using namespace std;


Queue * newQueue(int capacity){
	Queue * queue = new Queue;
	queue->capacity = capacity+1;
	queue->list = new AS_NodePointer[queue->capacity];
	queue->index = 1;
	return queue;
}

void Queue_insert(Queue * queue, AS_NodePointer node){
	if(queue->index >= queue->capacity){
		queue->list = (AS_NodePointer *) realloc(queue->list, sizeof(AS_NodePointer)*(queue->capacity+AS_QUEUE_ADITIONAL_CAPACITY));
		if(!queue->list){
			fprintf(stderr, "Error on increasing queue capacity.\n");
			exit(1);			
		}
		queue->capacity += AS_QUEUE_ADITIONAL_CAPACITY;
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
	delete queue->list;
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
	delete list->nodes;
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
	AS_Node * parent;
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

AS_NodePointer * AS_search(AS_Config * config){
	ClosedSet * closedSet = newClosedSet(config->areSameStates, config->closedSetChunkSize);
	Queue * queue = newQueue(config->queueInitialCapacity);
	
	Queue_insert(queue, config->startNode);
	
	
	while(true){
		if(Queue_isEmpty(queue)){
			AS_freeTree(config->startNode);
			return NULL;
		}
		
		AS_Node * node = Queue_remove(queue);
		
		if(config->isGoalState(node->state)){
			return AS_searchResult(node);
		}
		
		ClosedSet_add(closedSet, node);
		
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
	}
}

AS_Node * newASNode(double heuristic, double cost, AS_Node * parent){
	AS_Node * node = new AS_Node;
	node->data = NULL;
	node->state = NULL;
	node->parent = parent;
	node->heuristic = heuristic;
	node->cost = cost;
	node->status = AS_STATUS_IDLE;
	node->childrenLength = 0;
	node->children = NULL;
	return node;
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
	delete node;
}

void testQueue(){
	Queue * q = newQueue(1000);
	
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
