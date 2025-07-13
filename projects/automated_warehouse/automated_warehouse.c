#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "threads/init.h"
#include "threads/malloc.h"
#include "threads/synch.h"
#include "threads/thread.h"
#include "devices/timer.h"

#include "projects/automated_warehouse/aw_manager.h"
#include "projects/automated_warehouse/aw_message.h"
#include "projects/automated_warehouse/robot.h"
#include "projects/automated_warehouse/aw_thread.h"

#define CMD_WAIT 0
#define CMD_MOVE 1
#define CMD_DETOUR 2  // 새로운 명령 추가
#define MAX_ROBOTS 16

struct robot* robots;
extern int num_robots = 0;
int stuck_counter[MAX_ROBOTS];

// 충돌 판단 - 현재 위치 또는 다음 위치가 겹치면 true
bool is_occupied_by_other_robot(int r, int c, int current_robot_idx) {
    for (int j = 0; j < num_robots; j++) {
        if (j == current_robot_idx || boxes_from_robots[j].msg.done) continue;

        int cur_r = boxes_from_robots[j].msg.row;
        int cur_c = boxes_from_robots[j].msg.col;
        int nxt_r = boxes_from_robots[j].msg.next_row;
        int nxt_c = boxes_from_robots[j].msg.next_col;

        if ((cur_r == r && cur_c == c) || (nxt_r == r && nxt_c == c))
            return true;
    }
    return false;
}


// Deadlock 해소용 강제 이동 가능 여부: 다른 로봇의 next 위치만 고려
bool is_conflict_with_any_robot(int r, int c, int current_robot_idx) {
    for (int j = 0; j < num_robots; j++) {
        if (j == current_robot_idx || boxes_from_robots[j].msg.done) continue;

        int nr = boxes_from_robots[j].msg.next_row;
        int nc = boxes_from_robots[j].msg.next_col;

        // 출발도 안 한 로봇은 제외
        if (nr == boxes_from_robots[j].msg.row && nc == boxes_from_robots[j].msg.col) continue;

        if (nr == r && nc == c) return true;
    }
    return false;
}

bool can_force_move(int r, int c, int current_robot_idx) {
    for (int j = 0; j < num_robots; j++) {
        if (j == current_robot_idx || boxes_from_robots[j].msg.done) continue;

        // 현재 위치 검사
        if (boxes_from_robots[j].msg.row == r && boxes_from_robots[j].msg.col == c)
            return false;

        // 다음 위치도 검사 (출발하지 않은 경우는 허용하지 않음)
        int nr = boxes_from_robots[j].msg.next_row;
        int nc = boxes_from_robots[j].msg.next_col;
        if (nr == r && nc == c)
            return false;
    }
    return true;
}


// 중앙 제어 노드
void control_node_main(void* aux) {
    int num_robots = *(int*)aux;
    bool robot_done[num_robots];
    for (int i = 0; i < num_robots; i++) {
        robot_done[i] = false;
        stuck_counter[i] = 0;
    }

    while (true) {
        // 1. 모든 로봇 dirtyBit 수신 대기
        int received = 0;
        while (received < num_robots) {
            received = 0;
            for (int i = 0; i < num_robots; i++) {
                if (robot_done[i] || boxes_from_robots[i].dirtyBit == 1)
                    received++;
            }
            thread_yield();
        }

        // 2. 시각화 + step 증가
        print_map(robots, num_robots);
        increase_step();

        // 3. 충돌 회피 및 명령 결정
        bool all_waiting = true;
        bool used[MAP_HEIGHT][MAP_WIDTH] = { false };
        int start_idx = step % num_robots;

        for (int i = 0; i < num_robots; i++) {
            if (!robot_done[i]) {
                int r = boxes_from_robots[i].msg.row;
                int c = boxes_from_robots[i].msg.col;
                used[r][c] = true;
            }
        }

        for (int offset = 0; offset < num_robots; offset++) {
            int i = (start_idx + offset) % num_robots;
            if (robot_done[i]) {
                boxes_from_central_control_node[i].msg.cmd = -1;
                continue;
            }
        
            // ✅ step 0이면, 딱 한 로봇만 이동 시도 (step % num_robots)
            if (step == 0 && i != step % num_robots) {
                boxes_from_central_control_node[i].msg.cmd = CMD_WAIT;
                continue;
            }
        
            int r = boxes_from_robots[i].msg.next_row;
            int c = boxes_from_robots[i].msg.next_col;
            int cr = boxes_from_robots[i].msg.row;
            int cc = boxes_from_robots[i].msg.col;
        
            if (is_occupied_by_other_robot(r, c, i) || used[r][c]) {
                boxes_from_central_control_node[i].msg.cmd = CMD_WAIT;
                stuck_counter[i]++;
            } else {
                boxes_from_central_control_node[i].msg.cmd = CMD_MOVE;
                stuck_counter[i] = 0;
                used[r][c] = true;
            }
        
            if (boxes_from_central_control_node[i].msg.cmd == CMD_MOVE &&
                !(r == cr && c == cc)) {
                all_waiting = false;
            }
        }

        // 4. Deadlock 감지 시 강제 이동
        if (all_waiting) {
            for (int i = 0; i < num_robots; i++) {
                boxes_from_central_control_node[i].msg.cmd = CMD_WAIT;
                stuck_counter[i]++;
            }
        
            for (int offset = 0; offset < num_robots; offset++) {
                int i = (step + offset) % num_robots;
                if (robot_done[i]) continue;
        
                int r = boxes_from_robots[i].msg.next_row;
                int c = boxes_from_robots[i].msg.next_col;
        
                if (can_force_move(r, c, i)) {
                    boxes_from_central_control_node[i].msg.cmd = CMD_MOVE;
                    stuck_counter[i] = 0;
                    printf("[CTRL] 🚀 Forcing R%d to break out.\n", i + 1);
                    break;
                }
        
                // 우회 명령
                if (boxes_from_robots[i].msg.stage == 0) { // 출발도 못한 경우에만
                    boxes_from_central_control_node[i].msg.cmd = CMD_DETOUR;
                    printf("[CTRL] ↪️ Suggesting detour for R%d.\n", i + 1);
                }
            }
        }
        
        // 5. 완료 확인
        for (int i = 0; i < num_robots; i++) {
            if (!robot_done[i] && boxes_from_robots[i].msg.done == 1) {
                robot_done[i] = true;
                printf("[CTRL] ✅ R%d has completed delivery.\n", i + 1);
            }
        }

        // 6. 전체 완료 체크
        bool all_done = true;
        for (int i = 0; i < num_robots; i++) {
            if (!robot_done[i]) {
                all_done = false;
                break;
            }
        }

        if (all_done) {
            unblock_threads();
            printf("All robots have completed delivery. Shutting down simulation.\n");
            break;
        }

        // 7. dirtyBit 초기화 + unblock
        unblock_threads();
        for (int i = 0; i < num_robots; i++) {
            boxes_from_robots[i].dirtyBit = 0;
        }
    }
}



// 시뮬레이터 진입점
void run_automated_warehouse(char **argv) {
    init_automated_warehouse(argv);  // 초기화 (수정 금지)
    printf("implement automated warehouse!\n");

    // 1. 로봇 수 파싱
    num_robots = atoi(argv[1]);

    // 2. 동적 메모리 할당
    robots = malloc(sizeof(struct robot) * num_robots);
    boxes_from_robots = malloc(sizeof(struct messsage_box) * num_robots);
    boxes_from_central_control_node = malloc(sizeof(struct messsage_box) * num_robots);
    tid_t* threads = malloc(sizeof(tid_t) * (num_robots + 1));
    int *idxs = malloc(sizeof(int) * num_robots);

    if (!robots || !boxes_from_robots || !boxes_from_central_control_node || !threads || !idxs) {
        PANIC("Memory allocation failed in run_automated_warehouse");
    }

    for (int i = 0; i < num_robots; i++) {
        boxes_from_robots[i].dirtyBit = 0;
        boxes_from_central_control_node[i].dirtyBit = 0;
    }

    // 3. 로봇 task 정보 파싱
    char *task_str = argv[2];
    char *save_ptr;
    char *token = strtok_r(task_str, ":", &save_ptr);
    int idx = 0;

    while (token != NULL && idx < num_robots) {
        int item_no = token[0] - '0';
        char zone = token[1];
        char *name = malloc(4);
        snprintf(name, 4, "R%d", idx + 1);
        setRobot(&robots[idx], name, ROW_W, COL_W, item_no, 0, zone);
        idx++;
        token = strtok_r(NULL, ":", &save_ptr);
    }

    // 4. 중앙 제어 스레드 시작
    threads[0] = thread_create("CNT", 0, &control_node_main, &num_robots);

    // 5. 로봇 스레드 시작
    for (int i = 0; i < num_robots; i++) {
        idxs[i] = i;
        threads[i + 1] = thread_create(robots[i].name, 0, &robot_main, &idxs[i]);
    }
}
