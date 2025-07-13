#ifndef _PROJECTS_PROJECT1_ROBOT_H__
#define _PROJECTS_PROJECT1_ROBOT_H__
#include <stdbool.h> 

/**
 * A Structure representing robot
 */
struct robot {
    const char* name;
    int row, col;                // 현재 위치
    int next_row, next_col;      // 다음 위치 ← 추가 필요!!
    int required_payload;
    int current_payload;
    char unload_zone;
};

void setRobot(struct robot* _robot, const char* name, int row, int col, int required_payload, int current_payload, char unload_zone);

void robot_main(void* aux);

void control_node_main(void* aux);

bool bfs_path(int start_row, int start_col, int goal_row, int goal_col, int* next_row, int* next_col);

bool get_item_position(int item_no, int* row, int* col);

bool get_unload_zone_position(char zone, int* row, int* col);

char get_zone_from_robot_idx(int idx);

bool is_valid(int row, int col);

void recalculate_detour(struct robot* r); 


#endif