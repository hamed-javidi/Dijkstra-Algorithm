/*
 * Dijkstra Algorithm.h
 *
 *  Created on: May 8, 2020
 *      Author: hamed
 */

#ifndef DIJKSTRA_ALGORITHM_H_
#define DIJKSTRA_ALGORITHM_H_
#include <iostream>
#include <vector>
#include <queue>

class Node {
  int _id;
  float _cost;
 public:
//  Node(const Node &n) {
//    _id = n.id();
//    _cost = n.cost();
//  }
  Node(int id, float cost) {
    _id = id;
    _cost = cost;
  }
  int id() const {
    return _id;
  }
  float cost() const {
    return _cost;
  } 
  void SetCost(float cost) {
    _cost = cost;
  }
};

// To compare two Node
class myComparator {
 public:
  float operator()(const Node &n1, const Node &n2) {
    return n1.cost() > n2.cost();
  }
};

template<class T, class Container = std::vector<T>, class Compare = std::less<
    typename Container::value_type> >
class MyQueue : public std::priority_queue<T, Container, Compare> {
 public:
  typedef typename std::priority_queue<T, Container, Compare>::container_type::const_iterator const_iterator;

//  int find(const T &val) const {
//    auto first = this->c.cbegin();
//    auto last = this->c.cend();
//    while (first != last) {
//      if ((*first).id() == val.id())
//        return (*first).cost();
//      ++first;
//    }
//    return 0;
//  }
  const_iterator find(const T &val) const {
    auto first = this->c.cbegin();
    auto last = this->c.cend();
    while (first != last) {
      if ((*first).id() == val.id())
        return first;
      ++first;
    }
    return last;
  }
  bool remove(const T &value) {
    auto it = find(value);
    if (it != this->c.end()) {
      this->c.erase(it);
      std::make_heap(this->c.begin(), this->c.end(), this->comp);
      return true;
    } else {
      return false;
    }
  }
  bool CheckEnd(auto iter) {
    if (iter == this->c.cend())
      return false;
    return true;
  }
};

class Graph{
  //nodes labels if size = 5
  //    0  1  2  3  4
  //0 [ 0  0  1  1  0 ]
  //1 [ 0  0  1  1  1 ]
  //2 [ 0  0  0  0  0 ]
  //3 [ 0  0  0  0  1 ]
  //4 [ 0  0  0  0  0 ]
 public:
  Graph(uint8_t size, float density, float low_range, float high_range);
  ~Graph();
  int CountVertices(); 						//returns the number of vertices in the graph
  int CountEdges();									//returns the number of edges in the graph
  bool Adjacent(int node_x, int node_y); 	//tests whether there is an edge from node x to node y.
  std::vector<Node> Neighbors(Node node);  //lists all nodes y such that there is an edge from x to y.
  //void AddEdge(int node_x, int node_y, int cost);	//adds to G the edge from x to y, if it is not there.
  //void DeleteEdge(int node_x, int node_y);	// removes the edge from x to y, if it is there.
  //int GetEdgeCost(int node_x, int node_y);	// returns the value associated with the  edge (x,y)..
  //void SetEdgeCost(int node_x, int node_y, int cost);	//sets the value associated with the  edge (x,y).
  float** GetWeights() {
    return weight;
  }
 private:
  float **weight;
  uint8_t size;
  bool **edge;

};

class PriorityQueue {
 public:

  void removeTop() {                   //removes the top element of the queue.
    pq.pop();
  }
  float Contains(const Node node) {    //does the queue contain queue_element.
    auto it = pq.find(node);
    if (pq.CheckEnd(it))
      return it->id() == node.id() ? it->cost() : 0;
    else
      return 0;

  }
  void Insert(Node node) {            //insert queue_element into queue
    float temp = Contains(node);
    if (temp != 0 && temp > node.cost()) {
      pq.remove(node);
      pq.push(node);
    } else if (temp == 0) {
      pq.push(node);
    }
  }
  Node TopElement() {
    return pq.top();
  }                      //returns the top element of the queue.
  int QueueSize() {                       //return the number of queue_elements.
    return pq.size();
  }
 private:
  MyQueue<Node, std::vector<Node>, myComparator> pq;
};

class ShortestPath {
 public:

  std::vector<double> vertices(); 						//list of vertices in G(V,E).
  PriorityQueue path(Graph g, int u, int w);  // find shortest path between u-w and returns the sequence of vertices representing shorest path u-v1-v2-…-vn-w.
  int path_size(Graph g, int u, int w);  //return the path cost associated with the shortest path.
 private:
  PriorityQueue pq;

};


#endif /* DIJKSTRA_ALGORITHM_H_ */
