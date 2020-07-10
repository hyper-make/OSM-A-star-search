#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;
    
    start_node = &m_Model.FindClosestNode(start_x, start_y);
    end_node = &m_Model.FindClosestNode(end_x, end_y);
}

float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
	return node->distance(*end_node);
}

void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
	current_node->FindNeighbors();
    for (RouteModel::Node *neighbor : current_node->neighbors){
    	neighbor->parent = current_node;    
        neighbor->h_value = CalculateHValue(neighbor);
        neighbor->g_value = current_node->g_value + neighbor->distance(*current_node);
      	neighbor->visited = true;
        open_list.push_back(neighbor);
    }
}

bool Compare(RouteModel::Node *a, RouteModel::Node *b){
	float f1 = a->g_value + a->h_value;
    float f2 = b->g_value + b->h_value;
    return f1 > f2;
}

RouteModel::Node *RoutePlanner::NextNode() {
	sort(open_list.begin(), open_list.end(), Compare);
    RouteModel::Node *lowest = open_list.back();
    open_list.pop_back();
    return lowest;
}

std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector
    distance = 0;
    std::vector<RouteModel::Node> path_found;

    // TODO: Implement your solution here.
    // (current_node->distance(*start_node) != 0)
  	while (current_node->parent != nullptr){
        path_found.push_back(*current_node);
        distance += current_node->distance(*(current_node -> parent));
        current_node = current_node->parent;
    }
    path_found.push_back(*start_node);
    std::reverse(path_found.begin(), path_found.end());
  
    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    return path_found;

}

void RoutePlanner::AStarSearch() {
    RouteModel::Node *current_node = nullptr;

    current_node = start_node;
    current_node->h_value = CalculateHValue(current_node);
    current_node->g_value = 0;
    current_node->visited = true;   
    open_list.push_back(current_node);
  
  	while (current_node != end_node){
      current_node = NextNode();	
      AddNeighbors(current_node);
    }
  
  	m_Model.path = ConstructFinalPath(current_node);
  
    // TODO: Implement your solution here.

}
