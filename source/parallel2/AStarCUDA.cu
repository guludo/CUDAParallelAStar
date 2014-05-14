#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <float.h>
#include <assert.h>
#include "AStarCUDA.h"
#include "Queue.h"

#define NUM_BLOCKS 16
#define NUM_CHOICES 4
#define DUPLICATE 1

Queue * queue;


int dimension;

Queue * newQueue(int capacity){
	queue = new Queue;
	queue->capacity = capacity+1;
	queue->list = (AS_NodePointer *) malloc(sizeof(AS_NodePointer)*(queue->capacity));
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
	free(queue->list);
	delete queue;
}

void AS_initConfig(AS_Config * config){
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
int deb = 0;
void AS_freePath(AS_NodePointer * path){
	for(int i = 0; path[i]; i++){
		ASNode_free(path[i]);
	}
	delete [] path;
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
__device__ void expandNode_gpu(AS_Node * node, int size, int * states, char * fillResult, int dimension) {
	int l = dimension * dimension;
	int dy = (1 - threadIdx.x / 2) * (1 - (1 - threadIdx.x) * 2);
	int dx = (threadIdx.x / 2) * (1 - (3-threadIdx.x) * 2);
	int x = states[blockIdx.x * (l+3)+l] + dx;
	int y = states[blockIdx.x * (l+3)+l+1] + dy;
	if (x < 0 || x >= dimension || y < 0 || y >= dimension || (threadIdx.x + states[blockIdx.x * (l+3)+l+2] == 5) || (threadIdx.x + states[blockIdx.x*(l+3)+l+2] == 1)) 
		return;
	fillResult[blockIdx.x * blockDim.x + threadIdx.x] = 1;
}

__global__ void AS_search_gpu (AS_Node * nodes, int size, int * states, char * fillResult, int dim) {
	if (blockIdx.x >= size)
		return;
	expandNode_gpu(nodes, size, states, fillResult, dim);
}

__global__ void AS_remove_duplicate_gpu (AS_Node * nodes, int size, int * states, char * fillResult, int dim) {


	if (blockIdx.x >= threadIdx.x)
		return;

	if (threadIdx.y == NUM_CHOICES) {
		int l = dim * dim;
		int x1 = states[blockIdx.x * (l+3) + l];
		int y1 = states[blockIdx.x * (l+3) + l+1];
		int dy1 = (1 - blockIdx.y / 2) * (1 - (1 - blockIdx.y) * 2);
		int dx1 = (blockIdx.y / 2) * (1 - (3-blockIdx.y) * 2);
		for (int i = 0; i < l ; i++) {
			if (i != (x1 + dx1) * dim + y1 + dy1 && i != (x1 ) * dim + y1 && states[blockIdx.x * (l+3) + i] != 
				states[threadIdx.x * (l+3) + i])
				return;
		}
		fillResult[blockIdx.x * NUM_CHOICES + blockIdx.y] = 0;
		return;
	}

	if (fillResult[blockIdx.x * NUM_CHOICES + blockIdx.y] && fillResult[threadIdx.x * NUM_CHOICES + threadIdx.y] == 1) {
		int l = dim * dim;
		int x1 = states[blockIdx.x * (l+3) + l];
		int y1 = states[blockIdx.x * (l+3) + l+1];
		int dy1 = (1 - blockIdx.y / 2) * (1 - (1 - blockIdx.y) * 2);
		int dx1 = (blockIdx.y / 2) * (1 - (3-blockIdx.y) * 2);
		int x2 = states[threadIdx.x * (l+3) + l];
		int y2 = states[threadIdx.x * (l+3) + l+1];
		int dy2 = (1 - threadIdx.y / 2) * (1 - (1 - threadIdx.y) * 2);
		int dx2 = (threadIdx.y / 2) * (1 - (3-threadIdx.y) * 2);
		for (int i = 0; i < l; i++) {
			if (states[blockIdx.x * (l+3) + i] == 0 || states[blockIdx.x * (l+3) + i] == states[threadIdx.x * (l+3) + i])  
				continue;
			if (i == (x1 + dx1) * dim + y1 + dy1 && 
				(x1 +dx1 != x2 + dx2 || y1+dy1!=y2+dy2)) {
				return;
			}
			else if (i == x2 * dim + y2 && 
				states[blockIdx.x * (l+3) + i] != states[threadIdx.x * (l+3) + (x2 + dx2) * dim + y2 + dy2]) {
				return;
			} 
			return;
		}
		fillResult[threadIdx.x * NUM_CHOICES + threadIdx.y] = -1;
	}
}

AS_NodePointer * AS_search(AS_Config * config){
	// the solution path
	AS_NodePointer * path = NULL;
	
	Queue * queue = newQueue(config->queueInitialCapacity);
	Queue_insert(queue, config->startNode);
	int l = dimension * dimension;
	// whether path is found
	bool found = false;
	// size of the batch
	int nodeBatchSize;
	AS_Node ** nodes = (AS_Node **) malloc (sizeof(AS_Node*) * NUM_BLOCKS);	

	// pool for each batch
	// one batch of nodes retrieved from the heap
	AS_Node * nodeBatch = (AS_Node *) malloc (sizeof(AS_Node) * NUM_BLOCKS);
	// fill results of newly created nodes
	char * fillResult = (char *) malloc (sizeof(char) * NUM_BLOCKS * NUM_CHOICES);
	int * states = (int *) malloc (sizeof(int) * NUM_BLOCKS * (dimension * dimension + 3));


	// one batch of nodes and fill results in device
	AS_Node * d_nodeBatch;
	char * d_fillResult;
	int * d_states; 

	dim3 grid_block2D (NUM_BLOCKS, NUM_CHOICES);

	dim3 grid_thread2D (NUM_BLOCKS, NUM_CHOICES+1);

	int err = cudaMalloc((void **)&d_nodeBatch, sizeof(AS_Node) * NUM_BLOCKS);
	assert(err == cudaSuccess);
	
	err = cudaMalloc((void **)&d_fillResult, sizeof(char) * NUM_BLOCKS * NUM_CHOICES);
	assert(err == cudaSuccess);

	err = cudaMalloc((void **)&d_states, sizeof(int) * NUM_BLOCKS * (dimension * dimension + 3));
	assert(err == cudaSuccess);

	while (true) {
		// no path found
		printf("%d: queue size\n", queue->index);
		if(Queue_isEmpty(queue)){
			AS_freeTree(config->startNode);
			return NULL;
		}

		// switch between two different modes
		if (DUPLICATE) {
			nodeBatchSize = queue->index > NUM_BLOCKS ? NUM_BLOCKS : queue->index - 1;		

			for (int i = 0; i < nodeBatchSize; i++) {
				nodes[i] = Queue_remove(queue);
				if (config->isGoalState(nodes[i]->state)) {
					path = AS_searchResult(nodes[i]);
					found = true;
					break;
				}
				nodeBatch[i] = *(nodes[i]);
				//printf("node %d: blank at (%d %d), direction %d\n", i, nodes[i]->state[l], nodes[i]->state[l+1], nodes[i]->state[l+2]);
			}
		}
		else {
			int order = 0;
			while (!Queue_isEmpty(queue)) {
				nodes[order] = Queue_remove(queue);

				if (order == 0 || (order > 0 && !(config->areSameCost(nodes[order], nodes[order-1])))) {
					nodeBatch[order] = *(nodes[order]);
					order++;
				}
				else {
					int i = order-1;
					while (true) {
						if (config->areSameState(nodes[order]->state, nodes[i]->state)) {
							ASNode_free(nodes[order]);
							break;
						}
						i--;
						if (!(i >= 0 && config->areSameCost(nodes[order], nodes[i]))) {
							nodeBatch[order] = *(nodes[order]);
							order++;
							break;
						}
					}
				}
				if (order == NUM_BLOCKS)
					break;
			}
			nodeBatchSize = order;
			for (int i = 0; i < nodeBatchSize; i++) {
				if (config->isGoalState(nodes[i]->state)) {
					path = AS_searchResult(nodes[i]);
					found = true;
					break;
				}
			}
		}

		if (found)
			break;

		for (int i = 0; i < nodeBatchSize; i++) {
			cudaMemcpy(&d_states[i*(dimension * dimension + 3)], nodes[i] -> state, sizeof(int) * (dimension * dimension +3), 
				cudaMemcpyHostToDevice);
		}

		err = cudaMemcpy(d_nodeBatch, nodeBatch, sizeof(AS_Node) * nodeBatchSize, cudaMemcpyHostToDevice);
		assert(err == cudaSuccess);

		err = cudaMemset(d_fillResult, 0, sizeof(char) * NUM_BLOCKS * NUM_CHOICES);
		assert(err == cudaSuccess);

		// perform A* search
		AS_search_gpu <<<NUM_BLOCKS, NUM_CHOICES>>> (d_nodeBatch, nodeBatchSize, d_states, d_fillResult, dimension);

		// remove duplicates
		AS_remove_duplicate_gpu<<<grid_block2D, grid_thread2D>>> (d_nodeBatch, nodeBatchSize, d_states, d_fillResult, dimension);

		err = cudaMemcpy(fillResult, d_fillResult, sizeof(char) * NUM_BLOCKS * NUM_CHOICES, cudaMemcpyDeviceToHost);
		assert(err == cudaSuccess);

		
		for (int i =0; i < nodeBatchSize; i++) {
			int x = *((State *)(nodes[i]->state)+l);
			int y = *((State *)(nodes[i]->state)+l+1);
			for (int j = 0; j < NUM_CHOICES; j++) {
				//printf("%d: %d\n", j, fillResult[i * NUM_CHOICES + j]);
				if (fillResult[i * NUM_CHOICES + j] == 1) {
					int dy = (1 - j / 2) * (1 - (1 - j) * 2);
					int dx = (j / 2) * (1 - (3-j) * 2);
					State * cur = (State *) malloc (sizeof(State) * (l+3));
					for (int k = 0; k < l; k++) {
						cur[k] = *((State *)(nodes[i]->state)+k);
					}
					cur[x*dimension + y] = cur[(x+dx)*dimension + y + dy];
					cur[(x+dx)*dimension + y + dy] = 0;
					cur[l] = x+dx;
					cur[l+1] = y+dy;
					cur[l+2] = j;
					//printf("  (%d, %d)\n", dx, dy);
					AS_Node * created = newASNode(getHeuristic(cur));
					created->state = cur;
					Queue_insert(queue, created);
				}
			}
		}
	}

	return path;
}

double getHeuristic(State * state){
	State * s = (State *) state;
	double h = 0;
		for(int i = 0; i<dimension; i++) {
			for (int j = 0; j<dimension; j++) {
				int order = i * dimension + j;
				if(s[order] != 0) 
					h += abs(i - (s[order]-1)/dimension) + abs(j - (s[order]-1)%dimension);
			}
		}

	return h;
}


AS_Node * createNode(State * oldState, int swapIndexA, int swapIndexB){
	State * state = (State *) malloc(sizeof(int)*dimension*dimension);
	memcpy(state, oldState, sizeof(int)*dimension*dimension);
	state[swapIndexA] = oldState[swapIndexB];
	state[swapIndexB] = oldState[swapIndexA];
	AS_Node * node = newASNode(getHeuristic(state));
	node->state = state;
	return node;
}

AS_Node * newASNode(double h, double cost, AS_Node * parent){
	AS_Node * node = new AS_Node;
	node->data = NULL;
	node->state = NULL;
	node->parent = parent;
	node->heuristic = h;
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
	//delete [] node->children;
	delete node;
}

void cleanMem() {
	Queue_free(queue);
}

void cleanPath(AS_NodePointer * path) {
	delete [] path[0]->children;
	delete path[0];

	for (int i = 1; path[i]; i++) {
		ASNode_free(path[i]);
	}
	delete [] path;
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
