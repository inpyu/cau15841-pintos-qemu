# 🏗️ 중앙대학교 운영체제 25-1: PintOS Thread 프로젝트

본 프로젝트는 **PintOS** 기반의 스레드 프로그래밍을 활용하여 **멀티 로봇 자동화 창고 시스템(Automated Warehouse)** 을 구현합니다. 여러 대의 로봇이 하나의 창고에서 아이템을 적재하여 지정된 하역 구역으로 운반하는 시뮬레이션입니다. 

## 🧩 프로젝트 개요

- **목표**: 로봇들이 교착 상태(Deadlock) 없이 충돌(Collision) 없이 각자의 작업을 완수하도록 스레드 간 동기화 및 경로 탐색 구현
- **플랫폼**: PintOS (x86 에뮬레이션 기반 OS 교육용 플랫폼)
- **핵심 구현**:
  - 경량 메시지 기반 통신
  - BFS 경로 탐색
  - 충돌 감지 및 회피
  - 교착 상태 감지 및 우회/강제 이동 처리
  - 스레드 블로킹/언블로킹 처리 (`sema_down`, `sema_up` 활용)

## 🗂️ 디렉토리 구조

```bash
automated_warehouse/
├── automated_warehouse.c        # 시뮬레이터 진입점 및 제어 스레드 로직
├── automated_warehouse.h        # 시뮬레이터 함수 선언 및 유틸
├── aw_manager.h                 # 맵 정보 및 시각화 관련 상수
├── aw_message.c / .h            # 로봇 ↔ 제어 노드 간 메시지 구조체
├── aw_thread.c / .h             # 블로킹/언블로킹 스레드 관리
├── robot.c / .h                 # 개별 로봇 스레드 및 경로 탐색
````

## 🚀 실행 방법

### 1. 컴파일

```bash
cd threads
make
```

### 2. 실행

```bash
pintos -q run 'automated_warehouse 3 2A:3B:1C'
```

* `3`: 로봇 수
* `2A:3B:1C`: 각 로봇의 작업 (ex. 2번 아이템을 적재 → A구역 하역)

## 🤖 로봇 동작 흐름

### 단계별 작동

| 단계 (Stage) | 설명            |
| ---------- | ------------- |
| 0          | 아이템 적재 위치로 이동 |
| 1          | 하역 구역으로 이동    |
| 2          | 작업 완료 및 종료    |

### 명령 타입 (`cmd`)

| 명령           | 설명                      |
| ------------ | ----------------------- |
| `CMD_WAIT`   | 대기                      |
| `CMD_MOVE`   | 다음 위치로 이동               |
| `CMD_DETOUR` | 우회 경로 시도 (Deadlock 회피용) |

## 🧠 Deadlock/Collision 처리 전략

* **충돌 감지**: 다른 로봇의 현재 및 다음 위치와 중복 여부 판단
* **Deadlock 감지**: 모든 로봇이 동시에 이동 불가능 상태일 때
* **우회 로직**: 출발 전인 로봇에게 `CMD_DETOUR` 지시
* **강제 이동**: 충돌이 없을 경우 특정 로봇에게 이동 허용

## 🔄 메시지 구조 (`message`)

```c
struct message {
    int row, col;
    int current_payload, required_payload;
    int cmd;            // 제어 노드가 전달할 명령
    int next_row, next_col;
    int done;           // 1 = 완료
    int stage;          // 현재 단계
};
```

## 📌 주요 함수

| 함수                             | 역할             |
| ------------------------------ | -------------- |
| `robot_main()`                 | 개별 로봇 스레드 로직   |
| `control_node_main()`          | 중앙 제어 스레드      |
| `bfs_path()`                   | BFS 기반 경로 탐색   |
| `is_occupied_by_other_robot()` | 충돌 여부 판단       |
| `can_force_move()`             | 강제 이동 가능 여부 판단 |
| `recalculate_detour()`         | 우회 경로 재계산 로직   |

## 📷 시각화 예시

```plaintext
X    X    A    X    X    X    X
X    1         2    3    4    X
B                        R1   X
X                        R2   X
X    5         6    7    S    X
X    X    C    X    X    W    X
```

* `X`: 벽
* `A`, `B`, `C`, `S`: 하역 구역
* 숫자: 아이템 번호
* `W`: 출발지점
* `R1`, `R2`, ...: 로봇

## 📚 참고

* PintOS 환경 설정 및 기본 구조는 학교 측에서 제공한 소스코드를 기반으로 함
* 테스트 및 디버깅은 QEMU + Serial 콘솔 기반으로 진행
