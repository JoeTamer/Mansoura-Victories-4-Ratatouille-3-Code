#ifndef INC_CORE_COREHEADERFILES_FLOODFILL_H_
#define INC_CORE_COREHEADERFILES_FLOODFILL_H_

#include <core/coreHeaderFiles/movement.h>

extern int MAZE_MAP[16][16];
extern int FLOOD_MAP[16][16];

extern int currX, currY;

// === BFS Flood Fill ===
#define MAX_SIZE 256
typedef struct { int x, y; } Point;

// Initialize flood map
void fillMap(int returnHome);
// === MOVEMENT STATE ===
void updateCurr();
int checkNext(int dir);
void turn(int angle);
void proceed();

// === SENSOR LOGGING ===
void logEyes();
void sendMazeMap();

// === BFS Flood Fill ===
void queueInit();
void enqueue(int x, int y);
Point dequeue();
int isEmpty();
void floodfill(int returnHome);

#endif /* INC_CORE_COREHEADERFILES_FLOODFILL_H_ */
