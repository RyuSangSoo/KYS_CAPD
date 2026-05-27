#include <iostream>
#include <vector>
#include <queue>
#include <cmath>
#include <random>
#include <fstream>
#include <chrono>
#include <algorithm>

using namespace std;

struct Node {
    int x, y;
    double g, f;

    bool operator>(const Node& other) const {
        return f > other.f;
    }
};

struct Point {
    int x, y;
};

double heuristic(int x1, int y1, int x2, int y2) {
    return hypot(x1 - x2, y1 - y2);
}

bool inBounds(int x, int y, int w, int h) {
    return x >= 0 && x < w && y >= 0 && y < h;
}

vector<Point> astar(
    const vector<vector<int>>& grid,
    Point start,
    Point goal
) {
    int h = grid.size();
    int w = grid[0].size();

    vector<vector<double>> gScore(h, vector<double>(w, 1e9));
    vector<vector<Point>> parent(h, vector<Point>(w, {-1, -1}));
    vector<vector<bool>> closed(h, vector<bool>(w, false));

    priority_queue<Node, vector<Node>, greater<Node>> open;

    gScore[start.y][start.x] = 0.0;
    open.push({start.x, start.y, 0.0, heuristic(start.x, start.y, goal.x, goal.y)});

    int dx[8] = {1, -1, 0, 0, 1, 1, -1, -1};
    int dy[8] = {0, 0, 1, -1, 1, -1, 1, -1};

    while (!open.empty()) {
        Node cur = open.top();
        open.pop();

        if (closed[cur.y][cur.x]) continue;
        closed[cur.y][cur.x] = true;

        if (cur.x == goal.x && cur.y == goal.y) {
            vector<Point> path;
            Point p = goal;

            while (!(p.x == start.x && p.y == start.y)) {
                path.push_back(p);
                p = parent[p.y][p.x];
            }

            path.push_back(start);
            reverse(path.begin(), path.end());
            return path;
        }

        for (int i = 0; i < 8; i++) {
            int nx = cur.x + dx[i];
            int ny = cur.y + dy[i];

            if (!inBounds(nx, ny, w, h)) continue;
            if (grid[ny][nx] == 1) continue;
            if (closed[ny][nx]) continue;

            double moveCost = (i < 4) ? 1.0 : sqrt(2.0);
            double newG = gScore[cur.y][cur.x] + moveCost;

            if (newG < gScore[ny][nx]) {
                gScore[ny][nx] = newG;
                parent[ny][nx] = {cur.x, cur.y};

                double f = newG + heuristic(nx, ny, goal.x, goal.y);
                open.push({nx, ny, newG, f});
            }
        }
    }

    return {};
}

vector<vector<int>> generateRandomMap(int w, int h, double obstacleRatio) {
    vector<vector<int>> grid(h, vector<int>(w, 0));

    random_device rd;
    mt19937 gen(rd());
    uniform_real_distribution<> dis(0.0, 1.0);

    for (int y = 0; y < h; y++) {
        for (int x = 0; x < w; x++) {
            if (dis(gen) < obstacleRatio) {
                grid[y][x] = 1;
            }
        }
    }

    return grid;
}

void savePPM(
    const vector<vector<int>>& grid,
    const vector<Point>& path,
    Point start,
    Point goal,
    const string& filename
) {
    int h = grid.size();
    int w = grid[0].size();

    vector<vector<int>> pathMap(h, vector<int>(w, 0));
    for (auto& p : path) {
        pathMap[p.y][p.x] = 1;
    }

    ofstream file(filename);
    file << "P3\n";
    file << w << " " << h << "\n";
    file << "255\n";

    for (int y = 0; y < h; y++) {
        for (int x = 0; x < w; x++) {
            if (x == start.x && y == start.y) {
                file << "0 255 0 ";      // start: green
            } else if (x == goal.x && y == goal.y) {
                file << "255 0 0 ";      // goal: red
            } else if (pathMap[y][x]) {
                file << "0 0 255 ";      // path: blue
            } else if (grid[y][x] == 1) {
                file << "0 0 0 ";        // obstacle: black
            } else {
                file << "255 255 255 ";  // free: white
            }
        }
        file << "\n";
    }

    file.close();
}

void saveMapPathCSV(
    const vector<vector<int>>& grid,
    const vector<Point>& path,
    Point start,
    Point goal,
    const string& filename
) {
    ofstream file(filename);

    if (!file.is_open()) {
        cerr << "Failed to open file: " << filename << endl;
        return;
    }

    int h = grid.size();
    int w = grid[0].size();

    file << "x,y,type\n";

    // obstacle cells
    for (int y = 0; y < h; y++) {
        for (int x = 0; x < w; x++) {
            if (grid[y][x] == 1) {
                file << x << "," << y << ",obstacle\n";
            }
        }
    }

    // planned path cells
    for (const auto& p : path) {
        file << p.x << "," << p.y << ",path\n";
    }

    // start and goal
    file << start.x << "," << start.y << ",start\n";
    file << goal.x << "," << goal.y << ",goal\n";

    file.close();
}

int main() {
    int width = 100;
    int height = 100;

    Point start = {2, 2};
    Point goal = {width - 3, height - 3};

    int trials = 50;

    ofstream csv("../data/astar_perf.csv");
    csv << "obstacle_ratio,trial,success,path_length,planning_time_ms\n";

    for (double obstacleRatio = 0.05; obstacleRatio <= 0.40; obstacleRatio += 0.05) {
        for (int trial = 0; trial < trials; trial++) {

            auto grid = generateRandomMap(width, height, obstacleRatio);

            grid[start.y][start.x] = 0;
            grid[goal.y][goal.x] = 0;

            auto t1 = chrono::high_resolution_clock::now();

            vector<Point> path = astar(grid, start, goal);

            auto t2 = chrono::high_resolution_clock::now();
            double ms = chrono::duration<double, milli>(t2 - t1).count();

            int success = !path.empty();
            int pathLength = success ? path.size() : 0;

            csv << obstacleRatio << ","
                << trial << ","
                << success << ","
                << pathLength << ","
                << ms << "\n";

            if (trial == 0) {
                string ppm_filename = "../data/astar_ratio_" + to_string(obstacleRatio) + ".ppm";
                savePPM(grid, path, start, goal, ppm_filename);

                string csv_filename = "../data/astar_map_path_ratio_" + to_string(obstacleRatio) + ".csv";
                saveMapPathCSV(grid, path, start, goal, csv_filename);
            }
        }

        cout << "Finished obstacle ratio: " << obstacleRatio << endl;
    }

    csv.close();

    cout << "Saved performance data: ../data/astar_perf.csv\n";
    cout << "Saved sample map images: ../data/astar_ratio_*.ppm\n";
    cout << "Saved map/path CSV files: ../data/astar_map_path_ratio_*.csv\n";

    return 0;
}