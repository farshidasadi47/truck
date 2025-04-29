#ifndef KDTREE_H
#define KDTREE_H
#include <algorithm>
#include <math.h>
#include <vector>
/****************** Type defs *************************************************/
// A structure to represent node of kd tree
typedef struct tnode{
  std::vector<float> point; // To store k dimensional point
  tnode *left, *right;
} tnode;
/****************** Function declarations *************************************/
// Make the KD tree from given points based on first 2 axes
tnode* get_kd_tree(std::vector<std::vector<float>> points, int depth);
// Gives the distance between given node and target
float get_distance(std::vector<float> node_point, std::vector<float> target);
// Gives the closest node between two given node and the target
tnode* get_closest(tnode* n1, tnode* n2, std::vector<float> target);
// Gives closest node to the target, bruteforce search
std::vector<float> get_nn(std::vector<std::vector<float>> points, std::vector<float> target);
// Gives closest node to the target, KD tree approach
tnode* get_kd_nn_node(tnode* root, std::vector<float> target, int depth);
// Gives closest point to the target
std::vector<float> get_local(tnode* root, std::vector<float> target);
#endif // KDTREE_H
/****************** Implementation ********************************************/
/*
#include <iostream>
#include <algorithm>
#include <math.h>
#include <vector>
#include <chrono>
int main(){
  tnode *root = NULL;
  std::vector<float> target;
  std::vector<float> min_point;
  auto begin = std::chrono::high_resolution_clock::now();
  auto end = std::chrono::high_resolution_clock::now();
  auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - begin);
  printf("%25s%10d ms\n","Brute force: ",(int)elapsed.count());
  srand((unsigned) time(0));
  // Producing target
  target = {((float)rand()/RAND_MAX)*20-10, ((float)rand()/RAND_MAX)*20-10};
  // Producing points and kd_tree
  std::vector<std::vector<float>> points(100,std::vector<float>(2));
  for(int i=0;i<(int)points.size();i++){
    for(int j = 0;j<(int)points[0].size();j++){
      points[i][j] = ((float)rand()/RAND_MAX)*20-10;
    }
  }
  begin = std::chrono::high_resolution_clock::now();
  for(int i=0;i<100;i++){
    root = get_kd_tree(points,0);
  }
  end = std::chrono::high_resolution_clock::now();
  elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - begin);
  printf("%25s%8.2f ms\n","Node gen: ",(float)elapsed.count()/100);
  /////////////////////////////
  min_point = get_nn(points,target);
  printf("%25s%+8.2f, %+8.2f\n","min from brute force: ",min_point[0],min_point[1]);
  min_point = get_kd_nn(root,target,0)->point;
  printf("%25s%+8.2f, %+8.2f\n","min from kd-tree: ",min_point[0],min_point[1]);
  printf(" ******************** \n");

}
*/
