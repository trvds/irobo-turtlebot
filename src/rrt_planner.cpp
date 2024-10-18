
#include <rrt_planner/rrt_planner.h>

namespace rrt_planner {


    RRTPlanner::RRTPlanner(costmap_2d::Costmap2DROS *costmap, 
            const rrt_params& params) : params_(params), collision_dect_(costmap) {

        costmap_ = costmap->getCostmap();
        map_width_  = costmap_->getSizeInMetersX();
        map_height_ = costmap_->getSizeInMetersY();

        random_double_x.setRange(-map_width_, map_width_);
        random_double_y.setRange(-map_height_, map_height_);

        nodes_.reserve(params_.max_num_nodes);
    }

    bool RRTPlanner::planPath() {

        // clear everything before planning
        nodes_.clear();

        // Start Node
        createNewNode(start_, -1);

        double *p_rand, *p_new;
        Node nearest_node;

        for (unsigned int k = 1; k <= params_.max_num_nodes; k++) {

            p_rand = sampleRandomPoint();
            nearest_node = nodes_[getNearestNodeId(p_rand)];
            p_new = extendTree(nearest_node.pos, p_rand); // new point and node candidate

            if (!collision_dect_.obstacleBetween(nearest_node.pos, p_new)) {
                createNewNode(p_new, nearest_node.node_id);

            } else {
                continue;
            }

            if(k > params_.min_num_nodes) {
                
                if(computeDistance(p_new, goal_) <= params_.goal_tolerance){
                    return true;
                }
            }
        }

        return false;
    }

    int RRTPlanner::getNearestNodeId(const double *point) {
	double min_dist = std::numeric_limits<double>::max();
	int nearest_node_id = -1;
	
	for(size_t i = 0; i < nodes_.size();i++) {
	   double dist = computeDistance(nodes_[i].pos,point);
	   if(dist < min_dist){
	      min_dist = dist;
	      nearest_node_id = i;
	   }
	}
	
	return nearest_node_id;
        /**************************
         * Implement your code here
         **************************/

    }

    void RRTPlanner::createNewNode(const double* pos, int parent_node_id) {

        Node new_node(pos, nodes_.size(),parent_node_id);
        nodes_.emplace_back(new_node);
        /**************************
         * Implement your code here
         **************************/

       
        
    }

    double* RRTPlanner::sampleRandomPoint() {

        /**************************
         * Implement your code here
         **************************/
        double goal_region_radius = 5.0;
	double bias_probability = 0.2;
	if(random_double_x.generate() < bias_probability){
	    ROS_INFO("Biasing towards goal");
	    rand_point_[0] = goal_[0] + random_double_x.generate() * goal_region_radius - goal_region_radius / 2;
	    rand_point_[1] = goal_[1] + random_double_y.generate() * goal_region_radius - goal_region_radius / 2;
	}else{
            rand_point_[0] = random_double_x.generate(); // ... ;
            rand_point_[1] = random_double_y.generate(); // ... ;
      	    ROS_INFO("Sampled Random Point (%f, %f)", rand_point_[0], rand_point_[1]);
      	}
        return rand_point_;
    }
    
    
    double* RRTPlanner::extendTree(const double* point_nearest, const double* point_rand) {

        /**************************
         * Implement your code here
         **************************/
	double direction[2] = { point_rand[0] - point_nearest[0], point_rand[1] - point_nearest[1] };
	double norm = computeDistance(point_nearest, point_rand);
	
        candidate_point_[0] = point_nearest[0] + (direction[0] / norm) * params_.step;
        candidate_point_[1] = point_nearest[1] + (direction[1] / norm) * params_.step;
	ROS_INFO("Extending Tree: Nearest (%f,%f) -> New Point (%f,%f)",point_nearest[0],point_nearest[1],candidate_point_[0], candidate_point_[1]);
      	
        return candidate_point_;
    }
    
     
    double RRTPlanner::computeDistance(const double*p1,const double*p2){
       return std::sqrt(std::pow(p1[0]-p2[0],2) + std::pow(p1[1]-p2[1],2));
    }

    const std::vector<Node>& RRTPlanner::getTree() {

        return nodes_;
    }

    void RRTPlanner::setStart(double *start) {

        start_[0] = start[0];
        start_[1] = start[1];
    }

    void RRTPlanner::setGoal(double *goal) {

        goal_[0] = goal[0];
        goal_[1] = goal[1];
    }

};
