#include "BSP.h"

int check_two_lines_collision(float line1_x1, float line1_y1, float line1_x2, float line1_y2, float line2_x1, float line2_y1, float line2_x2, float line2_y2);

void find_same_point(wall wall1, wall wall2, float* same_x, float* same_y){
    float a11 = wall1.x1 - wall1.x2;
    float a21 = wall1.y1 - wall1.y2;
    float a12 = wall2.x1 - wall2.x2;
    float a22 = wall2.y1 - wall2.y2;
    float c1 = wall2.x1 - wall1.x1;
    float c2 = wall2.y1 - wall1.y1;
    float det = a11*a22 - a12*a21;
    float det1 = c1*a22 - a12*c2;
    float t_param   = det1/det;
    float finded_x = wall1.x1 + t_param*(wall1.x1 - wall1.x2);
    float finded_y = wall1.y1 + t_param*(wall1.y1 - wall1.y2);
    *same_x = finded_x;
    *same_y = finded_y;
}

int check_point_position (wall splitter_wall, float point_x, float point_y){
    float normal_x = splitter_wall.normal_x;
    float normal_y = splitter_wall.normal_y;
    float scalar_mul = normal_x*(splitter_wall.x1 - point_x) + normal_y*(splitter_wall.y1 - point_y);
    if (scalar_mul >= 0)
        return 1;
    else
        return -1;

}
int check_wall_position (wall splitter_wall, wall checked_wall){
    int first_point_position = check_point_position(splitter_wall, checked_wall.x1, checked_wall.y1);
    int second_point_position = check_point_position(splitter_wall, checked_wall.x2, checked_wall.y2);
    if (first_point_position == 1 && second_point_position == 1)
        return 1;
    if (first_point_position == -1 && second_point_position == -1)
        return -1;
    if (first_point_position*second_point_position < 0)
        return 2;
    return 2;
}

bsp_tree_wall::bsp_tree_wall (const std::vector<wall> &level) {
    if (level.size() == 1){
        node_wall = level[0];
        front_bsp_tree = NULL;
        back_bsp_tree = NULL;
        return ;
    }

    vector<wall> front_walls;
    vector<wall> back_walls;
    wall splitter_wall = level[0];
    for (size_t i = 1; i < level.size(); ++i){
        wall checked_wall = level[i];
        std::cout << "i" << std::endl;
        int wall_position = check_wall_position(splitter_wall, checked_wall);
        if (wall_position == 1)
            front_walls.push_back(checked_wall);
        if (wall_position == -1)
            back_walls.push_back(checked_wall);
        if (wall_position == 2){
            wall new_wall1 = checked_wall;
            wall new_wall2 = checked_wall;
            float same_x;
            float same_y;
            find_same_point (splitter_wall, checked_wall, &same_x, &same_y);
            new_wall1. x1 = same_x;
            new_wall1. y1 = same_y;
            new_wall2. x2 = same_x;
            new_wall2. y2 = same_y;
            if (check_wall_position(splitter_wall, new_wall1) == 1){
                front_walls.push_back(new_wall1);
                back_walls.push_back(new_wall2);}
            else {
                front_walls.push_back(new_wall2);
                back_walls.push_back(new_wall1);
            } 

        
        }

    }
    node_wall = splitter_wall;
    if (back_walls.size() != 0)
        back_bsp_tree = new bsp_tree_wall(back_walls);
    else
        back_bsp_tree = NULL;

    if (front_walls.size() != 0)
        front_bsp_tree = new bsp_tree_wall(front_walls);
    else
        front_bsp_tree = NULL;
   
    return;


    
}

int check_line_and_wall_same_point (wall wall0, float vecx1, float vecx2, float vecy1, float vecy2){
    return check_two_lines_collision(vecx1, vecy1, vecx2, vecy2, wall0.x1, wall0.y1, wall0.x2, wall0.y2);
}

int check_wall_collision (wall wall0, float vecx1, float vecy1, float vecx2, float vecy2){
    wall epsilon_wall1(wall0.x1 + wall0.normal_x*epsilon, wall0.y1 + wall0.normal_y*epsilon, wall0.x2 + wall0.normal_x*epsilon, wall0.y2 + wall0.normal_y*epsilon );
    wall epsilon_wall2(wall0.x1 - wall0.normal_x*epsilon, wall0.y1 - wall0.normal_y*epsilon, wall0.x2 - wall0.normal_x*epsilon, wall0.y2 - wall0.normal_y*epsilon );
    wall epsilon_wall3(wall0.x2 + wall0.normal_x*epsilon, wall0.y2 + wall0.normal_y*epsilon, wall0.x2 - wall0.normal_x*epsilon, wall0.y2 - wall0.normal_y*epsilon );
    wall epsilon_wall4(wall0.x1 + wall0.normal_x*epsilon, wall0.y1 + wall0.normal_y*epsilon, wall0.x1 - wall0.normal_x*epsilon, wall0.y1 - wall0.normal_y*epsilon );
    if (check_line_and_wall_same_point(epsilon_wall1, vecx1, vecx2, vecy1, vecy2) == 1)
        return 1;
    if (check_line_and_wall_same_point(epsilon_wall2, vecx1, vecx2, vecy1, vecy2) == 1)
        return 1;
    if (check_line_and_wall_same_point(epsilon_wall3, vecx1, vecx2, vecy1, vecy2) == 1)
        return 1;
    if (check_line_and_wall_same_point(epsilon_wall4, vecx1, vecx2, vecy1, vecy2) == 1)
        return 1;

    return 0;}        

int check_two_lines_collision(float line1_x1, float line1_y1, float line1_x2, float line1_y2, float line2_x1, float line2_y1, float line2_x2, float line2_y2){
    float normal_x = line1_y2 - line1_y1;
    float normal_y = line1_x1 - line1_x2;
    float scalar_mul1 = (line2_x1 - line1_x1)*normal_x + (line2_y1 - line1_y1)*normal_y;  
    float scalar_mul2 = (line2_x2 - line1_x1)*normal_x + (line2_y2 - line1_y1)*normal_y; 
    normal_x = line2_y2 - line2_y1;
    normal_y = line2_x1 - line2_x2;
    float scalar_mul3 = (line1_x1 - line2_x1)*normal_x + (line1_y1 - line2_y1)*normal_y;  
    float scalar_mul4 = (line1_x2 - line2_x1)*normal_x + (line1_y2 - line2_y1)*normal_y;
    if ((scalar_mul1 >= 0) && (scalar_mul2 <= 0) && (scalar_mul3 >= 0) && (scalar_mul4 <= 0))
        return 1;
    if ((scalar_mul1 >= 0) && (scalar_mul2 <= 0) && (scalar_mul3 <= 0) && (scalar_mul4 >= 0))
        return 1;
    if ((scalar_mul1 <= 0) && (scalar_mul2 >= 0) && (scalar_mul3 <= 0) && (scalar_mul4 >= 0))
        return 1;
    if ((scalar_mul1 <= 0) && (scalar_mul2 >= 0) && (scalar_mul3 >= 0) && (scalar_mul4 <= 0))
        return 1;
    return 0;
}


int bsp_tree_wall:: check_level_collision(wall* collision_wall, int* number_of_collized_walls, float new_pos_x, float new_pos_y){
    if (check_wall_collision(node_wall, pos_x, pos_y, new_pos_x, new_pos_y) == 1){
        std::cout << "COLLISION!  "  << std::endl;
        *collision_wall = node_wall;
        *number_of_collized_walls = *number_of_collized_walls + 1;
    }
    else 
        std::cout << "no_collision" << std::endl;

    int position_flag  = check_point_position(node_wall, pos_x, pos_y);
    if (position_flag == 1){
        if (front_bsp_tree == NULL)
            return 0;
        else
            front_bsp_tree -> check_level_collision(collision_wall, number_of_collized_walls, new_pos_x, new_pos_y);
    }
    if (position_flag == -1){
        if (back_bsp_tree == NULL)
            return 0;
        else
            back_bsp_tree -> check_level_collision(collision_wall, number_of_collized_walls, new_pos_x, new_pos_y);
    }

    return 0;
}

void change_gamer_position(float delta_x, float delta_y){
    wall collized_wall;
    int number_of_collized_walls = 0;
    bsp_main_node->check_level_collision(&collized_wall, &number_of_collized_walls, pos_x + delta_x, pos_y + delta_y);
    if (number_of_collized_walls > 1)
        return;
    if (number_of_collized_walls == 0){
        pos_x = pos_x + delta_x;
        pos_y = pos_y + delta_y;
        return;
    }
    if (number_of_collized_walls == 1){
        float scalar_mul = delta_x*collized_wall.normal_x + delta_y*collized_wall.normal_y;
        pos_x = pos_x + delta_x - collized_wall.normal_x*scalar_mul;
        pos_y = pos_y + delta_y - collized_wall.normal_y*scalar_mul;
    }

}








