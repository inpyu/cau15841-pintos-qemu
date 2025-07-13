#include "projects/automated_warehouse/robot.h"
#include "projects/automated_warehouse/aw_thread.h"  
#include "projects/automated_warehouse/aw_message.h"  
#include "projects/automated_warehouse/aw_manager.h"
#include "projects/automated_warehouse/automated_warehouse.h" 


#include "threads/malloc.h"
#include <stdbool.h>

extern struct robot* robots;
extern unsigned int step; // 현재 step 정보를 외부에서 참조

#define MAX_H 10
#define MAX_W 10
#define CMD_WAIT 0
#define CMD_MOVE 1
#define CMD_DETOUR 2 

const int drow[4] = {-1, 1, 0, 0};
const int dcol[4] = {0, 0, -1, 1};

// visited[row][col]에 이전 위치 저장
int prev_row[MAX_H][MAX_W];
int prev_col[MAX_H][MAX_W];

/**
 * A function setting up robot structure
 */
void setRobot(struct robot* _robot, const char* name, int row, int col, int required_payload, int current_payload, char zone){
    _robot->name = name;
    _robot->row = row;
    _robot->col = col;
    _robot->next_row = row;  // 처음엔 현재 위치와 같게 설정
    _robot->next_col = col;
    _robot->required_payload = required_payload;
    _robot->current_payload = current_payload;
    _robot->unload_zone = zone;
}

void robot_main(void* aux) {
    int idx = *(int*)aux;
    struct robot* me = &robots[idx];
    int stage = 0;  // 0: 적재하러 감, 1: 하역장으로 감, 2: 완료

    while (1) {
        // 1. 현재 상태 보고
        boxes_from_robots[idx].msg.row = me->row;
        boxes_from_robots[idx].msg.col = me->col;
        boxes_from_robots[idx].msg.current_payload = me->current_payload;
        boxes_from_robots[idx].msg.required_payload = me->required_payload;
        boxes_from_robots[idx].msg.done = (stage == 2) ? 1 : 0;
        boxes_from_robots[idx].msg.stage = stage;
        boxes_from_robots[idx].dirtyBit = 1;

        printf("[R%d] Reporting: pos=(%d,%d), payload=%d, target=%d, stage=%d\n",
               idx + 1, me->row, me->col, me->current_payload, me->required_payload, stage);

        if (stage == 2) {
            printf("[R%d] Transport complete at (%d,%d). Exiting.\n", idx + 1, me->row, me->col);
            thread_exit();
        }

        // 2. 중앙 제어 노드의 명령 대기
        block_thread();

        // 💡 초기 step에서 idx == step % num_robots 일 때만 이동 (충돌 방지)
        if (step == 0 && idx != step % 3) {
            printf("[R%d] (step 0) skipping initial move to avoid collision.\n", idx + 1);
            boxes_from_robots[idx].msg.next_row = me->row;
            boxes_from_robots[idx].msg.next_col = me->col;
            continue;
        }

        // 3. 목적지 결정
        int goal_row, goal_col;
        bool found_goal = false;

        if (stage == 0)
            found_goal = get_item_position(me->required_payload, &goal_row, &goal_col);
        else if (stage == 1)
            found_goal = get_unload_zone_position(me->unload_zone, &goal_row, &goal_col);

        if (!found_goal) {
            printf("[R%d] ❌ Goal not found. Skipping.\n", idx + 1);
            continue;
        }

        // 4. 경로 계획
        int next_row, next_col;
        bool found_path = bfs_path(me->row, me->col, goal_row, goal_col, &next_row, &next_col);
        boxes_from_robots[idx].msg.next_row = found_path ? next_row : me->row;
        boxes_from_robots[idx].msg.next_col = found_path ? next_col : me->col;

        if (found_path)
            printf("[R%d]  Requesting move to (%d,%d)\n", idx + 1, next_row, next_col);
        else
            printf("[R%d] No path found — waiting.\n", idx + 1);

        // 5. 실제 명령 수행
        if (boxes_from_central_control_node[idx].msg.cmd == CMD_MOVE) {
            me->row = boxes_from_robots[idx].msg.next_row;
            me->col = boxes_from_robots[idx].msg.next_col;
            printf("[R%d] MOVE to (%d,%d)\n", idx + 1, me->row, me->col);

        } else if (boxes_from_central_control_node[idx].msg.cmd == CMD_DETOUR) {
            recalculate_detour(me);
            me->row = me->next_row;
            me->col = me->next_col;
            boxes_from_robots[idx].msg.next_row = me->next_row;
            boxes_from_robots[idx].msg.next_col = me->next_col;
            printf("[R%d] DETOUR: move to (%d,%d)\n", idx + 1, me->next_row, me->next_col);

        } else {
            printf("[R%d] WAIT — staying at (%d,%d)\n", idx + 1, me->row, me->col);
        }

        // 6. 도착 여부 처리
        if (me->row == goal_row && me->col == goal_col) {
            if (stage == 0) {
                me->current_payload = me->required_payload;
                stage = 1;
                printf("[R%d] Item %d picked up.\n", idx + 1, me->current_payload);
            } else if (stage == 1) {
                stage = 2;
                printf("[R%d] Delivered to zone %c.\n", idx + 1, me->unload_zone);
            }
        }
    }
}



void recalculate_detour(struct robot* r) {
    int options[4][2] = {
        {r->row - 1, r->col}, // 위
        {r->row + 1, r->col}, // 아래
        {r->row, r->col - 1}, // 왼쪽
        {r->row, r->col + 1}  // 오른쪽
    };

    // 1차 시도: 유효하고, 점유되지 않은 칸만 이동
    for (int i = 0; i < 4; i++) {
        int nr = options[i][0];
        int nc = options[i][1];
        if ((nr != r->row || nc != r->col) &&
            is_valid(nr, nc) &&
            !is_occupied_by_other_robot(nr, nc, r - robots)) {
            r->next_row = nr;
            r->next_col = nc;
            return;
        }
    }

    // ✅ 2차 시도 (조심!): 유효하지만 이미 점유된 곳은 X
    printf("[R%d] ❌ No safe detour. Staying put.\n", (int)(r - robots) + 1);
    r->next_row = r->row;
    r->next_col = r->col;
}





// 내부 함수: 위치 유효성 검사
bool is_valid(int row, int col) {
    if (row < 0 || col < 0 || row >= MAP_HEIGHT || col >= MAP_WIDTH)
        return false;
    char cell = map_draw_default[row][col];
    if (cell == 'X') return false;
    return true;
}



bool bfs_path(int s_row, int s_col, int g_row, int g_col, int* next_row, int* next_col) {
    bool visited[MAX_H][MAX_W] = {false};
    struct queue {
        int row, col;
    } q[128];

    int front = 0, rear = 0;
    q[rear++] = (struct queue){s_row, s_col};
    visited[s_row][s_col] = true;
    prev_row[s_row][s_col] = -1;

    while (front < rear) {
        struct queue cur = q[front++];

        if (cur.row == g_row && cur.col == g_col)
            break;

        for (int i = 0; i < 4; i++) {
            int nr = cur.row + drow[i];
            int nc = cur.col + dcol[i];

            if (is_valid(nr, nc) && !visited[nr][nc]) {
                visited[nr][nc] = true;
                prev_row[nr][nc] = cur.row;
                prev_col[nr][nc] = cur.col;
                q[rear++] = (struct queue){nr, nc};
            }
        }
    }

    // 목표까지 도달 못함
    if (!visited[g_row][g_col])
        return false;

    // 역추적하여 첫 스텝 추출
    int r = g_row, c = g_col;
    while (prev_row[r][c] != s_row || prev_col[r][c] != s_col) {
        int pr = prev_row[r][c];
        int pc = prev_col[r][c];
        r = pr;
        c = pc;
    }

    *next_row = r;
    *next_col = c;
    return true;
}

bool get_item_position(int item_no, int* row, int* col) {
    char target = '0' + item_no;
    for (int r = 0; r < MAP_HEIGHT; r++) {
        for (int c = 0; c < MAP_WIDTH; c++) {
            if (map_draw_default[r][c] == target) {
                *row = r;
                *col = c;
                return true;
            }
        }
    }
    return false;
}

bool get_unload_zone_position(char zone, int* row, int* col) {
    for (int r = 0; r < MAP_HEIGHT; r++) {
        for (int c = 0; c < MAP_WIDTH; c++) {
            if (map_draw_default[r][c] == zone) {
                *row = r;
                *col = c;
                return true;
            }
        }
    }
    return false;
}


char get_zone_from_robot_idx(int idx) {
    return robots[idx].unload_zone;  // 이미 구조체 안에 있음
}


