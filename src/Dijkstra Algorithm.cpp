//============================================================================
// Name        : Dijkstra.cpp
// Author      : Hamed
// Version     :
// Copyright   : Your copyright notice
// Description : Hello World in C++, Ansi-style
//============================================================================

#include <iostream>
#include <vector>
#include "Dijkstra Algorithm.h"
#include <iomanip>
#include <math.h>


Graph::Graph(uint8_t size, float density, float fMin, float fMax) {

	srand(time(0));
	this->size=size;
	// heap 2D bool
	edge = new bool * [size];
	weight = new float * [size];
	for(int i =0; i < size; ++i){
		edge[i] = new bool[size];
		weight[i] = new float[size];
	}
	//initialize edges using density

	for(int i = 0; i < size; ++i){
		for(int j = i; j < size; ++j){

			if(i==j){
				edge[i][j] = false;
				weight[i][j] = 0;
			}
			else{
				if(density==0)
					edge[i][j] = false;
				else if (density == 1)
					edge[i][j] = true;
        else {

          double f = (double) rand() / RAND_MAX;
          edge[i][j] = f < density;
          if (edge[i][j]) {
            f = (double) rand() / RAND_MAX;
            weight[i][j] = fMin + f * (fMax - fMin);
          }
          else
            weight[i][j] = 0;
        }
			}
		}
	}
}
Graph::~Graph(){
//	for(int i =0; i < this->size; ++i){
//		delete weight[i];
//		delete edge[i];
//	}
//	delete weight;
//	delete edge;
}
int Graph::CountVertices(){
	return size;
}
int Graph::CountEdges(){
	int sum =0;
	for(int i = 0; i < size; ++i){
		for(int j = i; j < size; ++j){
			if(i != j && edge[i][j]){
				sum++;
			}
		}
	}
	return sum;
}
bool Graph::Adjacent(int node_x, int node_y) {
  if (node_x == node_y)
    return false;
  if (node_x < node_y)
    return edge[node_x][node_y];
  else
    return edge[node_y][node_x];
}
std::vector<Node> Graph::Neighbors(Node node) {
  std::vector<Node> neighbors;

  for (int i = 0; i < size; ++i) {
    if (edge[node.id()][i]) {
      Node temp(i, node.cost() + weight[node.id()][i]);
      neighbors.push_back(temp);
    }
  }
  return neighbors;
}

PriorityQueue ShortestPath::path(Graph g, int u, int w) {
  PriorityQueue close_set, open_set;
  Node node(u, 0);
  close_set.Insert(node);
  do {

    //step 1:
    for (auto i : g.Neighbors(node)) {
        open_set.Insert(i);
    }
    if (open_set.QueueSize() == 0)
      break;
    //step 2:
    node = open_set.TopElement();
    close_set.Insert(node);
    open_set.removeTop();
    if (node.id() == w)
      return close_set;

  } while (1);

  return open_set;
}

int main() {
  const int size = 50;
  std::vector<float> dense = { 0.2, 0.4 };
  for (int idx = 0; idx < dense.size(); ++idx) {
    const float density = dense[idx], min_weight = 1.0, max_weight = 10.0;
    int orig = 0, dest = size - 1;
    Graph g(size, density, min_weight, max_weight);
//    float **weights = g.GetWeights();
//    std::cout << std::fixed << std::setprecision(1);
//    for (int i = 0; i < size; ++i) {
//      for (int j = 0; j < size; ++j) {
//        std::cout << weights[i][j] << ", ";
//      }
//      std::cout << std::endl;
//    }
    ShortestPath sp;
    double sum = 0;
    int count = 0;
    for (int k = 1; k < size; ++k) {

      auto p = sp.path(g, orig, k);
      if (p.QueueSize() == 0)
        continue;
      while (p.QueueSize() > 0) {
        if (p.TopElement().id() == dest) {
          sum += p.TopElement().cost();
          count++;
        }
        p.removeTop();
      }
    }
    double avg = 0;
    if (count)
      avg = sum / count;
    std::cout << "Average cost for " << density << " : " << avg
              << std::endl;
  }
	return 0;
}
