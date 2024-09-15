#include<iostream>
#include<vector>
#include<random>
#include<ctime>
#include<stack>
#include<queue>

#include <chrono>
#include <thread>

std::vector<std::vector<int>> generateMatrix();

std::vector<std::vector<int>> generate_obstacle_Matrix(int m, int n);

bool isValid(int Point_Row, int Point_Col, int  m, int n,
            std::vector<std::vector<bool>>& visited,
            std::vector<std::vector<int>>& obstacle_matrix );

bool DFS_Connect(int startRow, int startCol,int endRow, int endCol,
                std::vector<std::vector<bool>>& visited,std::vector<std::vector<int>>& obstacle_matrix);

bool BFS_Connect(int startRow, int startCol,int endRow, int endCol,
                std::vector<std::vector<bool>>& visited,std::vector<std::vector<int>>& obstacle_matrix);

void Djt_shortest_path(int startRow, int startCol,int endRow, int endCol,
                std::vector<std::vector<bool>>& visited,std::vector<std::vector<int>>& obstacle_matrix);

int chebyshevDistance(int curX, int curY,int endRow, int endCol);

void Astar_shortest_path(int startRow, int startCol,int endRow, int endCol,
                std::vector<std::vector<bool>>& visited,std::vector<std::vector<int>>& obstacle_matrix);