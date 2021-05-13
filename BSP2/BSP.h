

#include <iostream>
#include <vector>
#include <math.h>
#define epsilon 0.018
float pos_x = 0.0;
float pos_y = 0.0;
using std::vector;
class wall{
public:
    float x1;
    float x2;
    float y1;
    float y2;
    float normal_x;
    float normal_y; 
    wall() = default;
    wall (float new_x1, float new_y1, float new_x2, float new_y2){
        x1 = new_x1;
        y1 = new_y1;
        x2 = new_x2;
        y2 = new_y2;
        float norm = sqrt((x1 - x2)*(x1 - x2) + (y1 - y2)*(y1 - y2));
        if ( (x1 - x2) >= 0 ){
            normal_y = (x1 - x2)/norm;
            normal_x = (y2 - y1)/norm;
        } 
        else {
            normal_y = (x2 - x1)/norm;
            normal_x = (y1 - y2)/norm;
        }

    }
private:
};
class bsp_tree_wall{
public:
    wall node_wall;
    bsp_tree_wall* front_bsp_tree;
    bsp_tree_wall* back_bsp_tree;
    bsp_tree_wall (const std::vector<wall> &level);
    int  check_level_collision(wall* collision_wall, int* number_of_collized_walls, float new_pos_x, float new_pos_y);
private:
} ;
bsp_tree_wall* bsp_main_node;