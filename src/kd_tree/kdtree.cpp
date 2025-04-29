#include "kdtree.h"
// Make the KD tree from given points based on first 2 axes
tnode* get_kd_tree(std::vector<std::vector<float>> points, int depth){
  // Getting number of axes
  int k = 2; // points[0].size();
  // Select axis based on current depth
  int axis = depth % k;
  // Sort points based on the current axis
  std::sort(std::begin(points), std::end(points),
    [=](const std::vector<float> vec1,const std::vector<float> vec2){return vec1[axis]<vec2[axis]; });
  // Find size of the points and median index
  int size_points = points.size();
  int median = size_points>1 ? (size_points+2-1)/2 - 1 : 0; // (m+n-1)/n -1 = ceil(m/n)-1
  // Creating and returning new node
  tnode* temp = new tnode; // makes this variable and allocate its size
  if (size_points < 2){
    // If only one point is left
    temp->point = points[0];
    temp->left = NULL;
    temp->right = NULL;
  }else{
    // If more than one point is left
    if(median<1){
      // If there are two points left
      temp->point = points[0];
      temp->left = NULL;
      temp->right = get_kd_tree(std::vector<std::vector<float>> (points.begin()+median+1, points.end()), depth + 1);
      points[1];
    }else{
      // If more than two points left
      temp->point = points[median];
      temp->left = get_kd_tree(std::vector<std::vector<float>> (points.begin(), points.begin() + median), depth + 1);
      temp->right = get_kd_tree(std::vector<std::vector<float>> (points.begin()+median+1, points.end()), depth + 1);
    }
  }
  return temp;
}
// Gives the distance between given node and target
float get_distance(std::vector<float> node_point, std::vector<float> target){
  // returns distance between given node and the target point
  float distance = 0;
  for(int i = 0;i<2;i++){
    distance += pow(node_point[i] - target[i],2);
  }
  return sqrt(distance);
}
// Gives the closest node between two given node and the target
tnode* get_closest(tnode* n1, tnode* n2, std::vector<float> target){
  // returns the closest node to a point, between two given nodes
  if (n1 == NULL){return n2;}
	float d1 = get_distance(n1->point, target);
	float d2 = get_distance(n2->point, target);
	if (d1 < d2){
	  return n1;
	}else{
	  return n2;
	}
}
// Gives closest node to the target, KD tree approach
tnode* get_kd_nn_node(tnode* root, std::vector<float> target, int depth){
  // Check if it is last node and return it
	if (root == NULL){return NULL;}
	// Getting number of axes
  int k = 2;
  // Select axis based on current depth
  int axis = depth % k;
  // Creating branches
  tnode* next_branch = NULL;
  tnode* other_branch = NULL;
  if(target[axis]>=root->point[axis]){
    // If the target is in the right side of current node
    next_branch = root->right;
    other_branch = root->left;
  }else{
    // If the target is in left side of current node
    next_branch = root->left;
    other_branch = root->right;
  }
  // Recurse down the branch that's best according to the current depth
  tnode* temp = NULL;
  tnode* best = NULL;
  temp = get_kd_nn_node(next_branch, target, depth + 1);
  best = get_closest(temp,root,target);
  // Calculating distance to current hyperplane to search in the other branch
  float radius = get_distance(best->point, target);
  float dist = abs(target[axis] - root->point[axis]);
  // Search other branch if condition satisfied
	if (radius > dist){
	  temp = get_kd_nn_node(other_branch, target, depth + 1);
		best = get_closest(temp, best, target);
	}
	return best;
}

// Gives closest point to the target in the format
// [x, y, psir, s, e]
std::vector<float> get_local(tnode* root, std::vector<float> target){
  std::vector<float> min_point;
  std::vector<float> diff(2);
  float psir;
  float distance;
  min_point = get_kd_nn_node(root, target,0)->point;
  psir = min_point[2];
  distance = get_distance(min_point, target);
  diff[0] = target[0] - min_point[0]; // x component: vector from min_point to target
  diff[1] = target[1] - min_point[1]; // y component: vector from min_point to target
  distance = -sin(min_point[2])*diff[0] + cos(min_point[2])*diff[1];
  std::vector<float> final_point(6);
  final_point[0] = min_point[0];
  final_point[1] = min_point[1];
  final_point[2] = min_point[2];
  final_point[3] = min_point[3];
  final_point[4] = min_point[4];
  final_point[5] = distance;
  return final_point;
}

// Gives closest node to the target, bruteforce search
std::vector<float> get_nn(std::vector<std::vector<float>> points, std::vector<float> target){
  // Calculates minimum distance point from brute force search
  float psir;
  float distance = 3.0e+030;
  float new_distance = 0;
  std::vector<float> min_point;
  std::vector<float> diff(2);
  for(const auto& i_arr : points){
    new_distance = get_distance(i_arr, target);
    if(new_distance<=distance){
      distance = new_distance;
      min_point = i_arr;
    }
  }
  diff[0] = target[0] - min_point[0]; // x component: vector from min_point to target
  diff[1] = target[1] - min_point[1]; // y component: vector from min_point to target
  distance = -sin(min_point[2])*diff[0] + cos(min_point[2])*diff[1];
  std::vector<float> final_point(6);
  final_point[0] = min_point[0];
  final_point[1] = min_point[1];
  final_point[2] = min_point[2];
  final_point[3] = min_point[3];
  final_point[4] = min_point[4];
  final_point[5] = distance;
  return final_point;
}
