#ifndef __PROJECTS_AUTOMATED_WAREHOUSE_H__
#define __PROJECTS_AUTOMATED_WAREHOUSE_H__

void run_automated_warehouse(char **argv);

bool is_occupied_by_other_robot(int r, int c, int current_robot_idx);

#endif 
