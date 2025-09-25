#include <stdio.h>
#include <string.h>
#include <core/coreHeaderFiles/floodfill.h>

#define MOVE_FORWARD() remi(FULLSTEP)
#define TURN_LEFT()    remi(SPINLEFT)
#define TURN_RIGHT()   remi(SPINRIGHT)
#define WALL_FRONT()   ((walls & 0b0010) || (walls & 0b0100))
#define WALL_RIGHT()   ((walls & 0b0001))
#define WALL_LEFT()    ((walls & 0b1000))

// === MAZE + FLOOD MAPS ===
int MAZE_MAP[16][16] = {
    { 12,  8,  8,  8,  8,  8,  8,  8,  8,  8,  8,  8,  8,  8,  8,  9},
    {  4,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  1},
    {  4,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  1},
    {  4,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  1},
    {  4,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  1},
    {  4,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  1},
    {  4,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  1},
    {  4,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  1},
    {  4,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  1},
    {  4,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  1},
    {  4,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  1},
    {  4,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  1},
    {  4,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  1},
    {  4,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  1},
    {  4,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  1},
    {  6,  2,  2,  2,  2,  2,  2,  2,  2,  2,  2,  2,  2,  2,  2,  3}
};

int FLOOD_MAP[16][16] = {
    {  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0  ,0},
    {  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0  ,0},
    {  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0  ,0},
    {  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0  ,0},
    {  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0  ,0},
    {  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0  ,0},
    {  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0  ,0},
    {  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0  ,0},
    {  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0  ,0},
    {  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0  ,0},
    {  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0  ,0},
    {  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0  ,0},
    {  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0  ,0},
    {  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0  ,0},
    {  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0  ,0},
    {  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0  ,0}
};

// Initialize flood map
void fillMap(int returnHome) {
    for (int x = 0; x < 16; x++)
        for (int y = 0; y < 16; y++)
            FLOOD_MAP[x][y] = 255;

    if (!returnHome) {
        FLOOD_MAP[7][7] = FLOOD_MAP[7][8] = 0;
        FLOOD_MAP[8][7] = FLOOD_MAP[8][8] = 0;
    } else {
        FLOOD_MAP[0][0] = 0;
    }
}
// === BFS Flood Fill ===
enum compass { North = 1, East, South, West };
int orient = North;
int currX = 0, currY = 0;
int nextX = 0, nextY = 0;
// Queue and visited array
Point queue[MAX_SIZE];
int front = 0;
int rear = 0;
int visited[16][16]; // Assuming a 16x16 grid, adjust size as needed

void updateCurr() {
    switch (orient) {
    case North: currY++; break;
    case East:  currX++; break;
    case South: currY--; break;
    case West:  currX--; break;
    }
}

int checkNext(int dir) {
    nextX = currX;
    nextY = currY;

    switch(dir) {
    case North: dir = 0; break;
    case East : dir = 1; break;
    case West : dir =-1;break;
    }

    int nextDir = orient + dir;
 	if (nextDir == 0) nextDir = West;
 	if (nextDir == 5) nextDir = North;

    switch(nextDir) {
    case North: nextY++; break;
    case East:  nextX++; break;
    case South: nextY--; break;
    case West:  nextX--; break;
    }

    return FLOOD_MAP[nextX][nextY];
}

void turn(int angle) {
    if (angle == 90) {
        TURN_RIGHT(); orient++;
    } else if (angle == -90) {
        TURN_LEFT(); orient--;
    } else if (angle == 180) {
        TURN_RIGHT(); TURN_RIGHT(); orient += 2;
    }

    if (orient > 4) orient -= 4;
    if (orient < 1) orient += 4;
}


void proceed() {
    if      ((checkNext(North) <= FLOOD_MAP[currX][currY]) && (!WALL_FRONT())) { MOVE_FORWARD(); updateCurr(); }
    else if ((checkNext(East)  <= FLOOD_MAP[currX][currY]) && (!WALL_RIGHT())) {
    	if (WALL_LEFT()) {
    		turn( 90);
    		backCalibration(0);
    		MOVE_FORWARD(); updateCurr();
    	}
    	else {
    		turn( 90);
    		MOVE_FORWARD(); updateCurr();
    	}
    }
    else if ((checkNext(West)  <= FLOOD_MAP[currX][currY]) && (!WALL_LEFT())) {
    	if (WALL_RIGHT()) {
    		turn(-90);
    		backCalibration(0);
    		MOVE_FORWARD(); updateCurr();
    	}
    	else {
    		turn(-90);
    		MOVE_FORWARD(); updateCurr();
    	}
    }
    else {
    	turn(180);
    	backCalibration(0);
        MOVE_FORWARD(); updateCurr();
    }
}

// === SENSOR LOGGING ===
void logEyes() {
    if (WALL_FRONT()) {
        switch (orient) {
        case North: MAZE_MAP[currX][currY] |= (1<<0); if (currY<15) MAZE_MAP[currX][currY+1] |= (1<<2); break;
        case East:  MAZE_MAP[currX][currY] |= (1<<1); if (currX<15) MAZE_MAP[currX+1][currY] |= (1<<3); break;
        case South: MAZE_MAP[currX][currY] |= (1<<2); if (currY>0)  MAZE_MAP[currX][currY-1] |= (1<<0); break;
        case West:  MAZE_MAP[currX][currY] |= (1<<3); if (currX>0)  MAZE_MAP[currX-1][currY] |= (1<<1); break;
        }
    }
    if (WALL_RIGHT()) {
        switch (orient) {
        case North: MAZE_MAP[currX][currY] |= (1<<1); if (currX<15) MAZE_MAP[currX+1][currY] |= (1<<3); break;
        case East:  MAZE_MAP[currX][currY] |= (1<<2); if (currY>0)  MAZE_MAP[currX][currY-1] |= (1<<0); break;
        case South: MAZE_MAP[currX][currY] |= (1<<3); if (currX>0)  MAZE_MAP[currX-1][currY] |= (1<<1); break;
        case West:  MAZE_MAP[currX][currY] |= (1<<0); if (currY<15) MAZE_MAP[currX][currY+1] |= (1<<2); break;
        }
    }
    if (WALL_LEFT()) {
        switch (orient) {
        case North: MAZE_MAP[currX][currY] |= (1<<3); if (currX>0)  MAZE_MAP[currX-1][currY] |= (1<<1); break;
        case East:  MAZE_MAP[currX][currY] |= (1<<0); if (currY<15) MAZE_MAP[currX][currY+1] |= (1<<2); break;
        case South: MAZE_MAP[currX][currY] |= (1<<1); if (currX<15) MAZE_MAP[currX+1][currY] |= (1<<3); break;
        case West:  MAZE_MAP[currX][currY] |= (1<<2); if (currY>0)  MAZE_MAP[currX][currY-1] |= (1<<0); break;
        }
    }
}

void sendMazeMap() {
    uart_send("MAP_BEGIN\n");
    for (int y = 15; y >= 0; y--) {   // print top row first
        for (int x = 0; x < 16; x++) {
            char buf[8];
            sprintf(buf, "%02X ", MAZE_MAP[x][y]);
            uart_send(buf);
        }
        uart_send("\n");
    }
    uart_send("MAP_END\n");
}


void queueInit() {
    front = rear = 0;
    memset(visited, 0, sizeof(visited));
}

void enqueue(int x, int y) {
    queue[rear].x = x;
    queue[rear].y = y;
    rear = (rear + 1) % MAX_SIZE;
}
Point dequeue() {
    Point p = queue[front];
    front = (front + 1) % MAX_SIZE;
    return p;
}
int isEmpty() { return (front == rear); }

void floodfill(int returnHome) {
    logEyes();
    //sendMazeMap();

    fillMap(returnHome);
    queueInit();

    if (!returnHome) {
        enqueue(7,8); enqueue(8,7); enqueue(7,7); enqueue(8,8);
        visited[7][8] = visited[8][7] = visited[7][7] = visited[8][8] = 1;
    } else {
        enqueue(0,0);
        visited[0][0] = 1;
    }

    while (!isEmpty()) {
        Point cur = dequeue();
        int x = cur.x, y = cur.y, val = FLOOD_MAP[x][y];

        if (y<15 && !(MAZE_MAP[x][y] & (1<<0)) && FLOOD_MAP[x][y+1] > val+1) { FLOOD_MAP[x][y+1]=val+1; if(!visited[x][y+1]){enqueue(x,y+1); visited[x][y+1]=1;} }
        if (x<15 && !(MAZE_MAP[x][y] & (1<<1)) && FLOOD_MAP[x+1][y] > val+1) { FLOOD_MAP[x+1][y]=val+1; if(!visited[x+1][y]){enqueue(x+1,y); visited[x+1][y]=1;} }
        if (y>0  && !(MAZE_MAP[x][y] & (1<<2)) && FLOOD_MAP[x][y-1] > val+1) { FLOOD_MAP[x][y-1]=val+1; if(!visited[x][y-1]){enqueue(x,y-1); visited[x][y-1]=1;} }
        if (x>0  && !(MAZE_MAP[x][y] & (1<<3)) && FLOOD_MAP[x-1][y] > val+1) { FLOOD_MAP[x-1][y]=val+1; if(!visited[x-1][y]){enqueue(x-1,y); visited[x-1][y]=1;} }
    }

    proceed();
}
