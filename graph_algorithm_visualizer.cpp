#include <GL/glut.h>
#include <iostream>
#include <vector>
#include <queue>
#include <stack>
#include <cmath>
#include <cstdlib>
#include <ctime>
#include <algorithm>
#include <unistd.h> // for usleep

using namespace std;

const int N = 21; // Max 20 nodes (1-20)
const int WINDOW_WIDTH = 800;
const int WINDOW_HEIGHT = 600;
const int NODE_RADIUS = 20;
const int MIN_DISTANCE = 100; // Minimum distance between nodes
const int INF = 1e9;

vector<pair<int, int>> adj[N]; // {neighbor, weight}
int numNodes, numEdges, startNode, endNode;
bool visited[N];
int currentNode = -1;
int dist[N];
int parent[N];
vector<int> shortestPath;

struct Node {
    float x, y;
    int id;
};

Node nodes[N];
bool isPaused = false;
bool isRunning = false;
bool isStepMode = false;
bool shouldStep = false;
enum TraversalMode { NONE, BFS, DFS, DIJKSTRA };
TraversalMode currentMode = NONE;

// Queue and stack for traversal
queue<int> bfsQueue;
stack<int> dfsStack;

// Priority queue for Dijkstra: {distance, node}
priority_queue<pair<int, int>, vector<pair<int, int>>, greater<pair<int, int>>> dijkstraPQ;

// Bresenham's Circle Drawing Algorithm
void drawCircleBresenham(int xc, int yc, int r, float red, float green, float blue) {
    int x = 0, y = r;
    int d = 3 - 2 * r;

    auto putPixels = [&](int x, int y) {
        glColor3f(red, green, blue);
        glBegin(GL_POINTS);
        glVertex2i(xc + x, yc + y);
        glVertex2i(xc - x, yc + y);
        glVertex2i(xc + x, yc - y);
        glVertex2i(xc - x, yc - y);
        glVertex2i(xc + y, yc + x);
        glVertex2i(xc - y, yc + x);
        glVertex2i(xc + y, yc - x);
        glVertex2i(xc - y, yc - x);
        glEnd();
    };

    while (y >= x) {
        putPixels(x, y);
        x++;
        if (d > 0) {
            y--;
            d = d + 4 * (x - y) + 10;
        } else {
            d = d + 4 * x + 6;
        }
    }
}

// Fill circle (for solid nodes)
void fillCircle(int xc, int yc, int r, float red, float green, float blue) {
    glColor3f(red, green, blue);
    glBegin(GL_TRIANGLE_FAN);
    glVertex2f(xc, yc);
    for (int i = 0; i <= 360; i++) {
        float angle = i * 3.14159 / 180.0;
        glVertex2f(xc + r * cos(angle), yc + r * sin(angle));
    }
    glEnd();
}

// Bresenham's Line Drawing Algorithm
void drawLineBresenham(int x1, int y1, int x2, int y2) {
    int dx = abs(x2 - x1);
    int dy = abs(y2 - y1);
    int sx = (x1 < x2) ? 1 : -1;
    int sy = (y1 < y2) ? 1 : -1;
    int err = dx - dy;

    glColor3f(0.7, 0.7, 0.7);
    glBegin(GL_POINTS);

    while (true) {
        glVertex2i(x1, y1);

        if (x1 == x2 && y1 == y2) break;

        int e2 = 2 * err;
        if (e2 > -dy) {
            err -= dy;
            x1 += sx;
        }
        if (e2 < dx) {
            err += dx;
            y1 += sy;
        }
    }
    glEnd();
}

// Check if position is valid (not too close to other nodes)
bool isValidPosition(float x, float y, int count) {
    for (int i = 1; i < count; i++) {
        float dist = sqrt((x - nodes[i].x) * (x - nodes[i].x) +
                         (y - nodes[i].y) * (y - nodes[i].y));
        if (dist < MIN_DISTANCE) return false;
    }
    return true;
}

// Generate random positions for nodes
void generateNodePositions() {
    srand(time(0));
    int margin = NODE_RADIUS + 40;

    for (int i = 1; i <= numNodes; i++) {
        int attempts = 0;
        do {
            nodes[i].x = margin + rand() % (WINDOW_WIDTH - 2 * margin);
            nodes[i].y = margin + rand() % (WINDOW_HEIGHT - 2 * margin);
            attempts++;
        } while (!isValidPosition(nodes[i].x, nodes[i].y, i) && attempts < 1000);

        nodes[i].id = i;
    }
}

// Draw text at position
void drawText(float x, float y, const char* text) {
    glColor3f(0.0, 0.0, 0.0);
    glRasterPos2f(x - 5, y - 5);
    for (const char* c = text; *c != '\0'; c++) {
        glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18, *c);
    }
}

// Draw instructions
void drawInstructions() {
    glColor3f(0.0, 0.0, 0.0);
    glRasterPos2f(10, WINDOW_HEIGHT - 20);
    string mode = (currentMode == BFS) ? "BFS" :
                  (currentMode == DFS) ? "DFS" :
                  (currentMode == DIJKSTRA) ? "DIJKSTRA" : "NONE";
    string status = isPaused ? "PAUSED" : isStepMode ? "STEP MODE" : "RUNNING";
    string text = "Mode: " + mode + " | Status: " + status + " | [SPACE] Pause | [S] Step | [A] Auto | [R] Reset";
    for (char c : text) {
        glutBitmapCharacter(GLUT_BITMAP_9_BY_15, c);
    }

    // Show shortest path info for Dijkstra
    if (currentMode == DIJKSTRA && !isRunning && dist[endNode] != INF) {
        glRasterPos2f(10, WINDOW_HEIGHT - 40);
        string pathInfo = "Shortest distance from " + to_string(startNode) + " to " +
                         to_string(endNode) + ": " + to_string(dist[endNode]);
        for (char c : pathInfo) {
            glutBitmapCharacter(GLUT_BITMAP_9_BY_15, c);
        }
    }
}

// Display function
void display() {
    glClear(GL_COLOR_BUFFER_BIT);

    // Draw edges
    for (int i = 1; i <= numNodes; i++) {
        for (auto& edge : adj[i]) {
            int j = edge.first;
            int weight = edge.second;
            if (i < j) { // Draw each edge only once
                // Check if edge is in shortest path
                bool inPath = false;
                if (currentMode == DIJKSTRA && !shortestPath.empty()) {
                    for (int k = 0; k < shortestPath.size() - 1; k++) {
                        if ((shortestPath[k] == i && shortestPath[k+1] == j) ||
                            (shortestPath[k] == j && shortestPath[k+1] == i)) {
                            inPath = true;
                            break;
                        }
                    }
                }

                if (inPath) {
                    // Draw path edge in blue
                    glColor3f(0.0, 0.0, 1.0);
                    glLineWidth(3.0);
                    glBegin(GL_LINES);
                    glVertex2f(nodes[i].x, nodes[i].y);
                    glVertex2f(nodes[j].x, nodes[j].y);
                    glEnd();
                    glLineWidth(1.0);
                } else {
                    drawLineBresenham(nodes[i].x, nodes[i].y, nodes[j].x, nodes[j].y);
                }

                // Draw edge weight
                float midX = (nodes[i].x + nodes[j].x) / 2;
                float midY = (nodes[i].y + nodes[j].y) / 2;
                glColor3f(1.0, 0.0, 0.0);
                char weightText[5];
                sprintf(weightText, "%d", weight);
                glRasterPos2f(midX, midY);
                for (char* c = weightText; *c != '\0'; c++) {
                    glutBitmapCharacter(GLUT_BITMAP_HELVETICA_12, *c);
                }
            }
        }
    }

    // Draw nodes
    for (int i = 1; i <= numNodes; i++) {
        // Fill circle based on visited status
        if (visited[i]) {
            fillCircle(nodes[i].x, nodes[i].y, NODE_RADIUS, 0.0, 1.0, 0.0); // Green
        } else {
            fillCircle(nodes[i].x, nodes[i].y, NODE_RADIUS, 1.0, 1.0, 1.0); // White
        }

        // Highlight start and end nodes for Dijkstra
        if (currentMode == DIJKSTRA) {
            if (i == startNode) {
                drawCircleBresenham(nodes[i].x, nodes[i].y, NODE_RADIUS + 5, 0.0, 0.5, 1.0);
                drawCircleBresenham(nodes[i].x, nodes[i].y, NODE_RADIUS + 6, 0.0, 0.5, 1.0);
            }
            if (i == endNode) {
                drawCircleBresenham(nodes[i].x, nodes[i].y, NODE_RADIUS + 5, 1.0, 0.5, 0.0);
                drawCircleBresenham(nodes[i].x, nodes[i].y, NODE_RADIUS + 6, 1.0, 0.5, 0.0);
            }
        }

        // Draw border
        drawCircleBresenham(nodes[i].x, nodes[i].y, NODE_RADIUS, 0.0, 0.0, 0.0);

        // Draw red border for current node
        if (i == currentNode) {
            drawCircleBresenham(nodes[i].x, nodes[i].y, NODE_RADIUS + 3, 1.0, 0.0, 0.0);
            drawCircleBresenham(nodes[i].x, nodes[i].y, NODE_RADIUS + 4, 1.0, 0.0, 0.0);
        }

        // Draw node number
        char label[3];
        sprintf(label, "%d", i);
        drawText(nodes[i].x, nodes[i].y, label);

        // Draw distance for Dijkstra
        if (currentMode == DIJKSTRA && visited[i] && dist[i] != INF) {
            glColor3f(0.0, 0.0, 1.0);
            char distText[10];
            sprintf(distText, "d:%d", dist[i]);
            glRasterPos2f(nodes[i].x - 15, nodes[i].y + NODE_RADIUS + 15);
            for (char* c = distText; *c != '\0'; c++) {
                glutBitmapCharacter(GLUT_BITMAP_HELVETICA_10, *c);
            }
        }
    }

    // Draw instructions
    drawInstructions();

    glutSwapBuffers();
}

// BFS traversal step
void bfsStep() {
    if (bfsQueue.empty()) {
        isRunning = false;
        currentNode = -1;
        return;
    }

    currentNode = bfsQueue.front();
    bfsQueue.pop();

    if (!visited[currentNode]) {
        visited[currentNode] = true;
        glutPostRedisplay();
        usleep(300000); // 300ms delay

        for (auto& edge : adj[currentNode]) {
            int neighbor = edge.first;
            if (!visited[neighbor]) {
                bfsQueue.push(neighbor);
            }
        }
    }
}

// DFS traversal step
void dfsStep() {
    if (dfsStack.empty()) {
        isRunning = false;
        currentNode = -1;
        return;
    }

    currentNode = dfsStack.top();
    dfsStack.pop();

    if (!visited[currentNode]) {
        visited[currentNode] = true;
        glutPostRedisplay();
        usleep(300000); // 300ms delay

        // Push neighbors in reverse order for correct DFS order
        vector<pair<int, int>> neighbors = adj[currentNode];
        for (int i = neighbors.size() - 1; i >= 0; i--) {
            if (!visited[neighbors[i].first]) {
                dfsStack.push(neighbors[i].first);
            }
        }
    }
}

// Dijkstra's algorithm step
void dijkstraStep() {
    if (dijkstraPQ.empty()) {
        isRunning = false;
        currentNode = -1;

        // Reconstruct shortest path
        if (dist[endNode] != INF) {
            shortestPath.clear();
            int curr = endNode;
            while (curr != -1) {
                shortestPath.push_back(curr);
                curr = parent[curr];
            }
            reverse(shortestPath.begin(), shortestPath.end());
        }

        glutPostRedisplay();
        return;
    }

    pair<int, int> top = dijkstraPQ.top();
    dijkstraPQ.pop();
    int d = top.first;
    int u = top.second;

    currentNode = u;

    if (visited[u]) {
        return;
    }

    visited[u] = true;
    glutPostRedisplay();
    usleep(300000); // 300ms delay

    // Relax edges
    for (auto& edge : adj[u]) {
        int v = edge.first;
        int weight = edge.second;

        if (!visited[v] && dist[u] + weight < dist[v]) {
            dist[v] = dist[u] + weight;
            parent[v] = u;
            dijkstraPQ.push({dist[v], v});
        }
    }
}

// Timer function for automatic mode
void timer(int value) {
    if (isRunning && !isPaused && !isStepMode) {
        if (currentMode == BFS) {
            bfsStep();
        } else if (currentMode == DFS) {
            dfsStep();
        } else if (currentMode == DIJKSTRA) {
            dijkstraStep();
        }
        glutPostRedisplay();
    }

    if (isRunning) {
        glutTimerFunc(50, timer, 0);
    }
}

// Reset visualization
void reset() {
    for (int i = 0; i < N; i++) {
        visited[i] = false;
        dist[i] = INF;
        parent[i] = -1;
    }
    currentNode = -1;
    isRunning = false;
    isPaused = false;
    shortestPath.clear();

    while (!bfsQueue.empty()) bfsQueue.pop();
    while (!dfsStack.empty()) dfsStack.pop();
    while (!dijkstraPQ.empty()) dijkstraPQ.pop();

    glutPostRedisplay();
}

// Start BFS
void startBFS() {
    reset();
    currentMode = BFS;
    bfsQueue.push(startNode);
    isRunning = true;
    if (!isStepMode) {
        glutTimerFunc(50, timer, 0);
    }
}

// Start DFS
void startDFS() {
    reset();
    currentMode = DFS;
    dfsStack.push(startNode);
    isRunning = true;
    if (!isStepMode) {
        glutTimerFunc(50, timer, 0);
    }
}

// Start Dijkstra
void startDijkstra() {
    reset();
    currentMode = DIJKSTRA;
    dist[startNode] = 0;
    dijkstraPQ.push({0, startNode});
    isRunning = true;
    if (!isStepMode) {
        glutTimerFunc(50, timer, 0);
    }
}

// Keyboard handler
void keyboard(unsigned char key, int x, int y) {
    switch (key) {
        case ' ': // Pause/Resume
            if (isRunning) {
                isPaused = !isPaused;
                glutPostRedisplay();
            }
            break;
        case 's':
        case 'S': // Step mode
            isStepMode = true;
            shouldStep = true;
            if (isRunning && shouldStep) {
                if (currentMode == BFS) {
                    bfsStep();
                } else if (currentMode == DFS) {
                    dfsStep();
                } else if (currentMode == DIJKSTRA) {
                    dijkstraStep();
                }
                shouldStep = false;
                glutPostRedisplay();
            }
            break;
        case 'a':
        case 'A': // Automatic mode
            isStepMode = false;
            if (isRunning && isPaused) {
                isPaused = false;
            }
            break;
        case 'r':
        case 'R': // Reset
            reset();
            break;
        case 'b':
        case 'B': // Start BFS
            startBFS();
            break;
        case 'd':
        case 'D': // Start DFS
            startDFS();
            break;
        case 'j':
        case 'J': // Start Dijkstra
            startDijkstra();
            break;
        case 27: // ESC
            exit(0);
            break;
    }
}

// Initialize OpenGL
void init() {
    glClearColor(0.95, 0.95, 0.95, 1.0);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluOrtho2D(0, WINDOW_WIDTH, 0, WINDOW_HEIGHT);
    glPointSize(2.0);
}

int main(int argc, char** argv) {
    // Input graph
    cout << "Enter number of nodes (1 to 20): ";
    cin >> numNodes;

    if (numNodes < 1 || numNodes > 20) {
        cout << "Invalid number of nodes!" << endl;
        return 1;
    }

    cout << "Enter number of edges: ";
    cin >> numEdges;

    cout << "Enter edges (node1 node2 weight):" << endl;
    for (int i = 0; i < numEdges; i++) {
        int u, v, w;
        cin >> u >> v >> w;
        if (u >= 1 && u <= numNodes && v >= 1 && v <= numNodes) {
            adj[u].push_back({v, w});
            adj[v].push_back({u, w});
        }
    }

    cout << "Enter starting node: ";
    cin >> startNode;

    if (startNode < 1 || startNode > numNodes) {
        cout << "Invalid starting node!" << endl;
        return 1;
    }

    cout << "Enter ending node (for Dijkstra): ";
    cin >> endNode;

    if (endNode < 1 || endNode > numNodes) {
        cout << "Invalid ending node!" << endl;
        return 1;
    }

    // Initialize distances
    for (int i = 0; i < N; i++) {
        dist[i] = INF;
        parent[i] = -1;
    }

    // Generate node positions
    generateNodePositions();

    // Initialize GLUT
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB);
    glutInitWindowSize(WINDOW_WIDTH, WINDOW_HEIGHT);
    glutCreateWindow("Graph Traversal Visualizer - Press 'B' for BFS, 'D' for DFS, 'J' for Dijkstra");

    init();

    glutDisplayFunc(display);
    glutKeyboardFunc(keyboard);

    cout << "\n=== Controls ===" << endl;
    cout << "B: Start BFS" << endl;
    cout << "D: Start DFS" << endl;
    cout << "J: Start Dijkstra (Shortest Path)" << endl;
    cout << "SPACE: Pause/Resume" << endl;
    cout << "S: Step mode (press S repeatedly to step through)" << endl;
    cout << "A: Automatic mode" << endl;
    cout << "R: Reset" << endl;
    cout << "ESC: Exit" << endl;

    glutMainLoop();

    return 0;
}