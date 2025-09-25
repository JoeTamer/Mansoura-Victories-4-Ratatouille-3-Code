#include <limits.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <core/coreHeaderFiles/aStar.h>

//=============================================================================
// CONFIGURATION CONSTANTS
//=============================================================================

#define DEBUG_BUFFER_SIZE 256
#define BUFFER_SIZE 32
#define MAX_PATH_LENGTH 1024

//=============================================================================
// WALL DIRECTION CONSTANTS
//=============================================================================

#define WALL_NORTH  0x01
#define WALL_EAST   0x02
#define WALL_SOUTH  0x04
#define WALL_WEST   0x08

//=============================================================================
// ENUMERATIONS
//=============================================================================

typedef enum {
    NORTH = 0,
    EAST  = 1,
    SOUTH = 2,
    WEST  = 3
} Heading;

typedef enum {
    FORW = 0,
    RIGHT   = 1,
    LEFT    = 2,
    IDLE    = 3
} Action;

typedef enum {
    EXPLORATION,
    RETURN_TO_START,
    OPTIMAL_PATH,
    RETURN_FROM_OPTIMAL,
    MISSION_COMPLETE
} MissionState;

//=============================================================================
// DATA STRUCTURES
//=============================================================================

typedef struct {
    uint8_t x, y;
    uint16_t f_score;
} Node;

typedef struct {
    bool initialized;
    MissionState state;

    uint8_t maze_width;
    uint8_t maze_height;
    uint16_t maze_size;

    uint8_t current_x;
    uint8_t current_y;
    Heading current_heading;

    uint8_t target_x;
    uint8_t target_y;

    uint8_t** walls;
    uint8_t** visited;
    uint16_t** g_scores;
    uint8_t** predecessor_x;
    uint8_t** predecessor_y;
    uint8_t** in_closed_list;

    Node* open_list;
    int open_list_count;

    uint8_t* optimal_path_x;
    uint8_t* optimal_path_y;
    uint16_t optimal_path_length;
    uint16_t path_step;

    uint8_t* path_history_x;
    uint8_t* path_history_y;
    uint16_t path_history_length;
} AStarState;

//=============================================================================
// GLOBAL VARIABLES AND CONSTANTS
//=============================================================================

// Aligned with Heading enum: 0=NORTH, 1=EAST, 2=SOUTH, 3=WEST
static const int dx[] = {0, 1, 0, -1};
static const int dy[] = {1, 0, -1, 0};
static const int wall_bits[] = {WALL_NORTH, WALL_EAST, WALL_SOUTH, WALL_WEST};

static char debug_buffer[DEBUG_BUFFER_SIZE];

AStarState astar = {
    .initialized = false,
    .state = EXPLORATION,
    .maze_width = 0,
    .maze_height = 0,
    .maze_size = 0,
    .current_x = 0,
    .current_y = 0,
    .current_heading = NORTH,
    .target_x = 0,
    .target_y = 0,
    .path_step = 0,
    .optimal_path_length = 0,
    .path_history_length = 0,
    .walls = NULL, .visited = NULL, .g_scores = NULL,
    .predecessor_x = NULL, .predecessor_y = NULL, .in_closed_list = NULL,
    .open_list = NULL, .optimal_path_x = NULL, .optimal_path_y = NULL,
    .path_history_x = NULL, .path_history_y = NULL,
};

//=============================================================================
// UTILITY AND HELPER FUNCTIONS (API AND LOGGING)
//=============================================================================

int API_mazeWidth() { return mazeWidth; }
int API_mazeHeight() { return mazeHeight; }
bool API_wallFront() { return ((walls & (0b0010)) || (walls & (0b0100))); }
bool API_wallRight() { return (walls & (0b0001)); }
bool API_wallLeft() { return (walls & (0b1000)); }
void API_moveFORW() { remi(FULLSTEP); }
void API_turnRight() { remi(SPINRIGHT); }
void API_turnLeft() { remi(SPINLEFT); }
void API_log(const char* text) { /*uart_send("log %s\n", text);*/ }

//=============================================================================
// CORE LOGIC HELPER FUNCTIONS
//=============================================================================

static inline int isValid(int x, int y) {
    return (x >= 0 && x < astar.maze_width && y >= 0 && y < astar.maze_height);
}

static void* safe_malloc(size_t size) {
    void* ptr = malloc(size);
    if (!ptr) {
        API_log("Critical Error: Memory allocation failed!");
    }
    return ptr;
}

static uint8_t** allocate_2d_uint8_array(int width, int height) {
    uint8_t** array = (uint8_t**)safe_malloc(width * sizeof(uint8_t*));
    if (!array) return NULL;

    for (int i = 0; i < width; i++) {
        array[i] = (uint8_t*)safe_malloc(height * sizeof(uint8_t));
        if (!array[i]) {
            for (int j = 0; j < i; j++) free(array[j]);
            free(array);
            return NULL;
        }
    }
    return array;
}

static uint16_t** allocate_2d_uint16_array(int width, int height) {
    uint16_t** array = (uint16_t**)safe_malloc(width * sizeof(uint16_t*));
    if (!array) return NULL;

    for (int i = 0; i < width; i++) {
        array[i] = (uint16_t*)safe_malloc(height * sizeof(uint16_t));
        if (!array[i]) {
            for (int j = 0; j < i; j++) free(array[j]);
            free(array);
            return NULL;
        }
    }
    return array;
}

static void free_2d_uint8_array(uint8_t** array, int width) {
    if (!array) return;
    for (int i = 0; i < width; i++) free(array[i]);
    free(array);
}

static void free_2d_uint16_array(uint16_t** array, int width) {
    if (!array) return;
    for (int i = 0; i < width; i++) free(array[i]);
    free(array);
}

static void deallocate_all_memory(void) {
    free_2d_uint8_array(astar.walls, astar.maze_width);
    free_2d_uint8_array(astar.visited, astar.maze_width);
    free_2d_uint16_array(astar.g_scores, astar.maze_width);
    free_2d_uint8_array(astar.predecessor_x, astar.maze_width);
    free_2d_uint8_array(astar.predecessor_y, astar.maze_width);
    free_2d_uint8_array(astar.in_closed_list, astar.maze_width);

    free(astar.open_list);
    free(astar.optimal_path_x);
    free(astar.optimal_path_y);
    free(astar.path_history_x);
    free(astar.path_history_y);

    memset(&astar, 0, sizeof(AStarState));
}

void cleanupAStar(void) {
    if (!astar.initialized) return;

    deallocate_all_memory();

    astar.initialized = false;
    API_log("A* cleanup completed");
}

static void setWallInternal(int x, int y, int direction) {
    if (!isValid(x, y)) return;
    astar.walls[x][y] |= direction;

    if (direction == WALL_NORTH && isValid(x, y + 1)) astar.walls[x][y + 1] |= WALL_SOUTH;
    if (direction == WALL_SOUTH && isValid(x, y - 1)) astar.walls[x][y - 1] |= WALL_NORTH;
    if (direction == WALL_EAST && isValid(x + 1, y)) astar.walls[x + 1][y] |= WALL_WEST;
    if (direction == WALL_WEST && isValid(x - 1, y)) astar.walls[x - 1][y] |= WALL_EAST;
}

//=============================================================================
// ENHANCED WALL READING AND DEBUGGING FUNCTIONS
//=============================================================================

static void debugPrintWallState(void) {
    char wall_debug[256];
    snprintf(wall_debug, sizeof(wall_debug),
        "Position (%d,%d) Heading:%d - Front:%s Left:%s Right:%s",
        astar.current_x, astar.current_y, astar.current_heading,
        API_wallFront() ? "WALL" : "OPEN",
        API_wallLeft() ? "WALL" : "OPEN",
        API_wallRight() ? "WALL" : "OPEN");
    API_log(wall_debug);
}

static bool readWallWithRetry(bool (*sensor_func)(void), const char* sensor_name) {
    bool reading1 = sensor_func();
    bool dummy = API_wallFront();
    (void)dummy;
    bool reading2 = sensor_func();

    if (reading1 != reading2) {
        bool reading3 = sensor_func();
        char debug_msg[128];
        snprintf(debug_msg, sizeof(debug_msg),
            "Inconsistent %s readings: %d,%d,%d - using %d",
            sensor_name, reading1, reading2, reading3, reading3);
        API_log(debug_msg);
        return reading3;
    }

    return reading1;
}

void updateWalls(void) {
    if (!isValid(astar.current_x, astar.current_y)) {
        API_log("ERROR: Invalid current position for wall update");
        return;
    }

    astar.visited[astar.current_x][astar.current_y] = 1;

    debugPrintWallState();

    bool wall_front = readWallWithRetry(API_wallFront, "FRONT");
    bool wall_left = readWallWithRetry(API_wallLeft, "LEFT");
    bool wall_right = readWallWithRetry(API_wallRight, "RIGHT");

    int front_wall_idx = astar.current_heading;
    int right_wall_idx = (astar.current_heading + 1) % 4;
    int left_wall_idx = (astar.current_heading + 3) % 4;

    if (wall_front) setWallInternal(astar.current_x, astar.current_y, wall_bits[front_wall_idx]);
    if (wall_left)  setWallInternal(astar.current_x, astar.current_y, wall_bits[left_wall_idx]);
    if (wall_right) setWallInternal(astar.current_x, astar.current_y, wall_bits[right_wall_idx]);
}

//=============================================================================
// ENHANCED INITIALIZATION WITH VALIDATION
//=============================================================================

int initAStar(void) {
    if (astar.initialized) {
        API_log("Warning: A* already initialized");
        cleanupAStar();
    }

    astar.maze_width = API_mazeWidth();
    astar.maze_height = API_mazeHeight();

    if (astar.maze_width <= 0 || astar.maze_height <= 0 ||
        astar.maze_width > 255 || astar.maze_height > 255) {
        char error_msg[128];
        snprintf(error_msg, sizeof(error_msg),
            "ERROR: Invalid maze dimensions: %dx%d",
            astar.maze_width, astar.maze_height);
        API_log(error_msg);
        return 0;
    }

    astar.maze_size = astar.maze_width * astar.maze_height;

    astar.walls = allocate_2d_uint8_array(astar.maze_width, astar.maze_height);
    astar.visited = allocate_2d_uint8_array(astar.maze_width, astar.maze_height);
    astar.g_scores = allocate_2d_uint16_array(astar.maze_width, astar.maze_height);
    astar.predecessor_x = allocate_2d_uint8_array(astar.maze_width, astar.maze_height);
    astar.predecessor_y = allocate_2d_uint8_array(astar.maze_width, astar.maze_height);
    astar.in_closed_list = allocate_2d_uint8_array(astar.maze_width, astar.maze_height);

    astar.open_list = (Node*)safe_malloc(astar.maze_size * sizeof(Node));
    astar.optimal_path_x = (uint8_t*)safe_malloc(astar.maze_size * sizeof(uint8_t));
    astar.optimal_path_y = (uint8_t*)safe_malloc(astar.maze_size * sizeof(uint8_t));
    astar.path_history_x = (uint8_t*)safe_malloc(MAX_PATH_LENGTH * sizeof(uint8_t));
    astar.path_history_y = (uint8_t*)safe_malloc(MAX_PATH_LENGTH * sizeof(uint8_t));

    if (!astar.walls || !astar.visited || !astar.g_scores ||
        !astar.predecessor_x || !astar.predecessor_y || !astar.in_closed_list ||
        !astar.open_list || !astar.optimal_path_x || !astar.optimal_path_y ||
        !astar.path_history_x || !astar.path_history_y) {
        API_log("ERROR: Memory allocation failed during A* initialization");
        deallocate_all_memory();
        return 0;
    }

    for (int i = 0; i < astar.maze_width; i++) {
        for (int j = 0; j < astar.maze_height; j++) {
            astar.walls[i][j] = 0;
            astar.visited[i][j] = 0;
            astar.g_scores[i][j] = UINT16_MAX;
            astar.predecessor_x[i][j] = 255;
            astar.predecessor_y[i][j] = 255;
            astar.in_closed_list[i][j] = 0;
        }
    }

    for (int i = 0; i < astar.maze_width; i++) {
        setWallInternal(i, 0, WALL_SOUTH);
        setWallInternal(i, astar.maze_height - 1, WALL_NORTH);
    }
    for (int j = 0; j < astar.maze_height; j++) {
        setWallInternal(0, j, WALL_WEST);
        setWallInternal(astar.maze_width - 1, j, WALL_EAST);
    }

    astar.path_history_length = 1;
    astar.path_history_x[0] = 0;
    astar.path_history_y[0] = 0;

    astar.current_x = 0;
    astar.current_y = 0;
    astar.current_heading = NORTH;
    astar.open_list_count = 0;
    astar.state = EXPLORATION;

    astar.initialized = true;
    API_log("A* initialization successful with enhanced wall reading");

    updateWalls();

    return 1;
}

//=============================================================================
// PRIORITY QUEUE (MIN-HEAP) IMPLEMENTATION
//=============================================================================

static inline void swap_nodes(Node* a, Node* b) {
    Node temp = *a;
    *a = *b;
    *b = temp;
}

static void heap_sift_up(Node* heap, int index) {
    while (index > 0) {
        int parent_index = (index - 1) / 2;
        if (heap[index].f_score < heap[parent_index].f_score) {
            swap_nodes(&heap[index], &heap[parent_index]);
            index = parent_index;
        } else {
            break;
        }
    }
}

static void heap_sift_down(Node* heap, int count, int index) {
    int smallest = index;
    while (true) {
        int left_child = 2 * index + 1;
        int right_child = 2 * index + 2;

        if (left_child < count && heap[left_child].f_score < heap[smallest].f_score) {
            smallest = left_child;
        }
        if (right_child < count && heap[right_child].f_score < heap[smallest].f_score) {
            smallest = right_child;
        }

        if (smallest != index) {
            swap_nodes(&heap[index], &heap[smallest]);
            index = smallest;
        } else {
            break;
        }
    }
}

static void push_to_heap(Node* heap, int* count, Node node) {
    if (*count >= astar.maze_size) {
        API_log("Warning: Priority queue is full");
        return;
    }
    heap[*count] = node;
    heap_sift_up(heap, *count);
    (*count)++;
}

static Node pop_from_heap(Node* heap, int* count) {
    Node root = heap[0];
    (*count)--;
    heap[0] = heap[*count];
    heap_sift_down(heap, *count, 0);
    return root;
}

//=============================================================================
// A* ALGORITHM IMPLEMENTATION (REFACTORED)
//=============================================================================

static inline int calculateHeuristic(int x1, int y1, int x2, int y2) {
    return abs(x1 - x2) + abs(y1 - y2);
}

void setTarget(int x, int y) {
    if (isValid(x, y)) {
        astar.target_x = x;
        astar.target_y = y;
    } else {
        API_log("Warning: Invalid target coordinates specified");
    }
}

static inline int isAtTarget(int x, int y) {
    return (x == astar.target_x && y == astar.target_y);
}

static void resetAStarSearch(void) {
    if (!astar.initialized) return;
    for (int i = 0; i < astar.maze_width; i++) {
        for (int j = 0; j < astar.maze_height; j++) {
            astar.g_scores[i][j] = UINT16_MAX;
            astar.predecessor_x[i][j] = 255;
            astar.predecessor_y[i][j] = 255;
            astar.in_closed_list[i][j] = 0;
        }
    }
    astar.open_list_count = 0;
}

static int storeOptimalPath(int start_x, int start_y) {
    astar.optimal_path_length = 0;
    if (!isValid(start_x, start_y) || !isValid(astar.target_x, astar.target_y)) {
        API_log("Error: Invalid coordinates for path reconstruction");
        return 0;
    }

    int trace_x = astar.target_x;
    int trace_y = astar.target_y;
    int safety_counter = 0;
    const int MAX_ITERATIONS = astar.maze_size * 2;

    while (safety_counter < MAX_ITERATIONS && astar.optimal_path_length < astar.maze_size) {
        astar.optimal_path_x[astar.optimal_path_length] = trace_x;
        astar.optimal_path_y[astar.optimal_path_length] = trace_y;
        astar.optimal_path_length++;

        if (trace_x == start_x && trace_y == start_y) break;

        if (astar.predecessor_x[trace_x][trace_y] == 255) {
            API_log("Error: Broken path - missing predecessor");
            astar.optimal_path_length = 0;
            return 0;
        }

        int temp_x = astar.predecessor_x[trace_x][trace_y];
        int temp_y = astar.predecessor_y[trace_x][trace_y];
        trace_x = temp_x;
        trace_y = temp_y;
        safety_counter++;
    }

    if (astar.optimal_path_length == 0 || safety_counter >= MAX_ITERATIONS) {
        API_log("Error: Path reconstruction failed");
        astar.optimal_path_length = 0;
        return 0;
    }

    for (int i = 0; i < astar.optimal_path_length / 2; i++) {
        int j = astar.optimal_path_length - 1 - i;
        int temp_x = astar.optimal_path_x[i];
        int temp_y = astar.optimal_path_y[i];
        astar.optimal_path_x[i] = astar.optimal_path_x[j];
        astar.optimal_path_y[i] = astar.optimal_path_y[j];
        astar.optimal_path_x[j] = temp_x;
        astar.optimal_path_y[j] = temp_y;
    }

    snprintf(debug_buffer, DEBUG_BUFFER_SIZE, "Path reconstructed: %d steps", astar.optimal_path_length);
    API_log(debug_buffer);

    return 1;
}

int runAStar(int start_x, int start_y) {
    if (!astar.initialized || !isValid(start_x, start_y)) {
        API_log("Error: A* not initialized or invalid start coordinates");
        return 0;
    }

    resetAStarSearch();
    astar.g_scores[start_x][start_y] = 0;

    Node start_node = { .x = start_x, .y = start_y, .f_score = calculateHeuristic(start_x, start_y, astar.target_x, astar.target_y) };
    push_to_heap(astar.open_list, &astar.open_list_count, start_node);

    int nodes_processed = 0;
    const int MAX_NODES = astar.maze_size * 4;

    while (astar.open_list_count > 0 && nodes_processed < MAX_NODES) {
        Node current_node = pop_from_heap(astar.open_list, &astar.open_list_count);
        int x = current_node.x;
        int y = current_node.y;

        nodes_processed++;

        if (isAtTarget(x, y)) {
            snprintf(debug_buffer, DEBUG_BUFFER_SIZE, "A* found path in %d iterations", nodes_processed);
            API_log(debug_buffer);
            return 1;
        }

        if (astar.in_closed_list[x][y]) continue;
        astar.in_closed_list[x][y] = 1;

        for (int i = 0; i < 4; i++) {
            int nx = x + dx[i];
            int ny = y + dy[i];

            if (!isValid(nx, ny) || (astar.walls[x][y] & wall_bits[i])) {
                continue;
            }

            if ((astar.state == OPTIMAL_PATH || astar.state == RETURN_FROM_OPTIMAL) && !astar.visited[nx][ny]) {
                continue;
            }

            int tentative_g = astar.g_scores[x][y] + 1;

            if (tentative_g < astar.g_scores[nx][ny]) {
                astar.predecessor_x[nx][ny] = x;
                astar.predecessor_y[nx][ny] = y;
                astar.g_scores[nx][ny] = tentative_g;
                uint16_t f_score = tentative_g + calculateHeuristic(nx, ny, astar.target_x, astar.target_y);
                Node neighbor_node = { .x = nx, .y = ny, .f_score = f_score };
                push_to_heap(astar.open_list, &astar.open_list_count, neighbor_node);
            }
        }
    }

    if (nodes_processed >= MAX_NODES) {
        API_log("Warning: A* terminated due to iteration limit");
    }

    return 0;
}

//=============================================================================
// MOVEMENT AND NAVIGATION CONTROL
//=============================================================================

static int getRequiredTurns(Heading from, Heading to) {
    int diff = (to - from + 4) % 4;
    switch (diff) {
        case 0: return 0;
        case 1: return 1;
        case 2: return 2;
        case 3: return -1;
    }
    return 0;
}

static Heading getHeadingForDirection(int dx_val, int dy_val) {
    for (int i = 0; i < 4; i++) {
        if (dx[i] == dx_val && dy[i] == dy_val) {
            return (Heading)i;
        }
    }
    return NORTH;
}

static int findNextExplorationStep(int* next_x, int* next_y) {
    if (astar.g_scores[astar.target_x][astar.target_y] == UINT16_MAX) {
        API_log("No path found to target during exploration");
        return 0;
    }

    int trace_x = astar.target_x;
    int trace_y = astar.target_y;
    int safety_counter = 0;
    const int MAX_TRACE = astar.maze_size * 2;

    while (safety_counter < MAX_TRACE) {
        if (astar.predecessor_x[trace_x][trace_y] == astar.current_x &&
            astar.predecessor_y[trace_x][trace_y] == astar.current_y) {
            *next_x = trace_x;
            *next_y = trace_y;
            return 1;
        }

        if (astar.predecessor_x[trace_x][trace_y] == 255) {
            API_log("Broken path during exploration step finding");
            return 0;
        }

        int temp_x = astar.predecessor_x[trace_x][trace_y];
        int temp_y = astar.predecessor_y[trace_x][trace_y];
        trace_x = temp_x;
        trace_y = temp_y;
        safety_counter++;
    }

    API_log("Failed to find next exploration step");
    return 0;
}

//=============================================================================
// STATE MANAGEMENT AND MAIN CONTROL
//=============================================================================

static void calculateMazeCenter(int* center_x, int* center_y) {
    *center_x = astar.maze_width / 2;
    *center_y = astar.maze_height / 2;
    if (astar.maze_width % 2 == 0) (*center_x)--;
    if (astar.maze_height % 2 == 0) (*center_y)--;
}

static int handleExplorationPhase(int* next_x, int* next_y) {
    int center_x, center_y;
    calculateMazeCenter(&center_x, &center_y);

    if (astar.current_x == center_x && astar.current_y == center_y) {
        astar.state = RETURN_TO_START;
        API_log("Reached center - starting return to origin");
        return 0;
    }

    setTarget(center_x, center_y);
    if (!runAStar(astar.current_x, astar.current_y)) {
        API_log("No path found during exploration");
        return 0;
    }
    return findNextExplorationStep(next_x, next_y);
}

static int handleReturnToStartPhase(int* next_x, int* next_y) {
    if (astar.current_x == 0 && astar.current_y == 0) {
        astar.state = OPTIMAL_PATH;
        astar.path_step = 0;
        API_log("Returned to origin - starting optimal path phase");
        return 0;
    }

    setTarget(0, 0);
    if (!runAStar(astar.current_x, astar.current_y)) {
        API_log("No path found for returning to start");
        return 0;
    }
    return findNextExplorationStep(next_x, next_y);
}

static int handleOptimalPathPhase(int* next_x, int* next_y) {
    int center_x, center_y;
    calculateMazeCenter(&center_x, &center_y);
    setTarget(center_x, center_y);

    static bool path_calculated = false;
    if (!path_calculated) {
        if (!runAStar(astar.current_x, astar.current_y)) {
            API_log("No optimal path found - mission failed");
            astar.state = MISSION_COMPLETE;
            return 0;
        }
        if (!storeOptimalPath(astar.current_x, astar.current_y)) {
            API_log("Failed to store optimal path");
            astar.state = MISSION_COMPLETE;
            return 0;
        }
        path_calculated = true;
    }

    if (astar.path_step >= astar.optimal_path_length ||
        (astar.current_x == astar.target_x && astar.current_y == astar.target_y)) {
        astar.state = RETURN_FROM_OPTIMAL;
        API_log("Reached center via optimal path! Starting return to origin.");
        return 0;
    }

    if (astar.path_step < astar.optimal_path_length &&
            astar.optimal_path_x[astar.path_step] == astar.current_x &&
            astar.optimal_path_y[astar.path_step] == astar.current_y) {
        astar.path_step++;
    }

    if (astar.path_step >= astar.optimal_path_length) {
        astar.state = RETURN_FROM_OPTIMAL;
        API_log("Finished optimal path traversal. Starting return to origin.");
        return 0;
    }

    *next_x = astar.optimal_path_x[astar.path_step];
    *next_y = astar.optimal_path_y[astar.path_step];
    return 1;
}

static int handleReturnFromOptimalPhase(int* next_x, int* next_y) {
    if (astar.current_x == 0 && astar.current_y == 0) {
        astar.state = MISSION_COMPLETE;
        API_log("Mission Complete: Returned to origin and stopped.");
        return 0;
    }

    setTarget(0, 0);
    if (!runAStar(astar.current_x, astar.current_y)) {
        API_log("No path found to return to origin.");
        return 0;
    }
    return findNextExplorationStep(next_x, next_y);
}

//=============================================================================
// MAIN SOLVER FUNCTION
//=============================================================================

static void performAction(Action action) {
    switch (action) {
        case FORW:
            API_moveFORW();
            astar.current_x += dx[astar.current_heading];
            astar.current_y += dy[astar.current_heading];
            if (astar.path_history_length < MAX_PATH_LENGTH) {
                astar.path_history_x[astar.path_history_length] = astar.current_x;
                astar.path_history_y[astar.path_history_length] = astar.current_y;
                astar.path_history_length++;
            } else {
                API_log("Warning: Path history buffer full");
            }
            updateWalls();
            break;
        case LEFT:
            API_turnLeft();
            astar.current_heading = (astar.current_heading + 3) % 4;
            break;
        case RIGHT:
            API_turnRight();
            astar.current_heading = (astar.current_heading + 1) % 4;
            break;
        case IDLE:
            break;
    }
}

Action astarSolver(int* next_x, int* next_y) {
    if (!astar.initialized) {
        if (!initAStar()) {
            API_log("Failed to initialize A* - mission impossible");
            astar.state = MISSION_COMPLETE;
            return IDLE;
        }
    }

    if (astar.state == MISSION_COMPLETE) return IDLE;

    bool success = false;
    switch (astar.state) {
        case EXPLORATION:
        	SPEED = 200.0f;
            gpio_writePin(runLedPort, runLedPin, LOW);
            gpio_writePin(searchLedPort, searchLedPin, HIGH);
            success = handleExplorationPhase(next_x, next_y);
            break;
        case RETURN_TO_START:
        	SPEED = 200.0f;
            gpio_writePin(runLedPort, runLedPin, LOW);
            gpio_writePin(searchLedPort, searchLedPin, HIGH);
            success = handleReturnToStartPhase(next_x, next_y);
            break;
        case OPTIMAL_PATH:
        	SPEED = 225.0f;
            gpio_writePin(runLedPort, runLedPin, HIGH);
            gpio_writePin(searchLedPort, searchLedPin, LOW);
            success = handleOptimalPathPhase(next_x, next_y);
            break;
        case RETURN_FROM_OPTIMAL:
            gpio_writePin(runLedPort, runLedPin, LOW);
            gpio_writePin(searchLedPort, searchLedPin, HIGH);
            success = handleReturnFromOptimalPhase(next_x, next_y);
            break;
        case MISSION_COMPLETE:
            return IDLE;
    }

    if (!success) {
        API_log("A* failed to find a path in the current state.");
        return IDLE;
    }

    int dx_move = *next_x - astar.current_x;
    int dy_move = *next_y - astar.current_y;

    if (abs(dx_move) + abs(dy_move) != 1) {
        API_log("Error: Non-adjacent move attempted");
        return IDLE;
    }

    Heading required_heading = getHeadingForDirection(dx_move, dy_move);
    int turns = getRequiredTurns(astar.current_heading, required_heading);

    if (turns == 0) return FORW;
    if (turns == 1) return RIGHT;
    if (turns == -1) return LEFT;
    if (turns == 2) {
        performAction(LEFT);
        performAction(LEFT);
        return FORW;
    }
    return IDLE;
}

int aStar() {
    API_log("Running...");
    while (astar.state != MISSION_COMPLETE) {
        int next_x = -1, next_y = -1;
        Action nextMove = astarSolver(&next_x, &next_y);
        performAction(nextMove);
    }

    remi(HALT);
    core_speak(CELEBRATE1);
    return 0;
}
