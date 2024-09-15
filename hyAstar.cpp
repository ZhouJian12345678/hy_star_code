#include "hyAstar.hpp"
// #include<iostream>
// #include<vector>
// #include<random>
// #include<ctime>
// #include<stack>
// #include<queue>

// #include <chrono>
// #include <thread>

//  使用 lambda 表达式
auto MEASURE_TIME = [](auto func, auto... args) {
    auto start = std::chrono::high_resolution_clock::now();
    func(args...);
    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    std::cout <<  " func took " << duration.count() << " milliseconds." << std::endl;
};

// 1. 实现一个随机生成m*n矩阵的程序（也可以用GRID_HEIGHT行，GRID_WIDTH列）
std::vector<std::vector<int>> generateMatrix(){
    int m, n;
    std::cout << " Enter the number of rows (m) ";
    std::cin >> m;
    std::cout << " Enter the number of columns (n) ";
    std::cin >> n;
     // std::vector< std::vector<int> > matrix = generate_matrix();
    std::random_device rd;    // 这个实例在构造时会尝试填充其内部状态，以确保它可以产生随机数
    std::mt19937 gen(rd() ^ static_cast<unsigned int>(std::time(0)) );   // 标准随机数生成器,异或 std::random_device 的输出和当前时间的转换值
    std::uniform_int_distribution<> dis(1,50); // 定义随机数范围

    // 创建一个 m×n 的矩阵，来表示一个网格，使用 std::vector 的容器
    std::vector< std::vector<int> > matrix( m, std::vector<int>(n,0) );
    // 打印矩阵大小
    std::cout << "Matrix size: " << m << "x" << n << std::endl;
    
    // 然后给矩阵赋随机值  由于随机数生成器的伪随机性质，偶尔可能会遇到连续0的情况
    
    for (size_t i = 0; i < m; ++i) {
        for (size_t j = 0; j < n; ++j) {
            // gen.seed(rd()); // 重新播种
            matrix[i][j] = dis(gen); // 赋一个1-50之间的随机数
        }
    }
    // 打印矩阵
    for (const auto &row : matrix) {
        for (const auto &elem : row) {
            std::cout << elem << " ";
        }
        std::cout << std::endl;
    }
    return matrix;
}

// 2. 实现n个随机位置生成障碍物( 矩阵是mxn，外界输入)
    /*细分问题：
    （1）确定网格的大小。
    （2）使用随机数生成器来确定障碍物的位置。
    （3）确保障碍物不会重复放置在同一个位置
    */ 
std::vector<std::vector<int>> generate_obstacle_Matrix(int m, int n){

    std::random_device rd;    // 这个实例在构造时会尝试填充其内部状态，以确保它可以产生随机数
    std::mt19937 gen(rd() ^ static_cast<unsigned int>(std::time(0)) );   // 标准随机数生成器种子/引擎,异或 std::random_device 的输出和当前时间的转换值
    
    // 创建一个 m×n 的矩阵，表示一个网格，网格初始化为全1，表示没有障碍物
    std::vector< std::vector<int> > matrix( m, std::vector<int>(n,1) );
    // 打印矩阵大小
    std::cout << "Matrix size: " << m << "x" << n << std::endl;

    // 定义障碍物的数量
    int numObstacles = n;
    // 确保障碍物的数量不会超过网格大小
    if(numObstacles> m * n){
        std::cerr << " Too many obstacles for the grid size " << std::endl;
    }

    // 生成边的权重
     for (int i = 0; i < m * n; ++i) {
        std::uniform_int_distribution<> disvalue(1, 10);
        // int value = disvalue(gen); // 随机权重1到10
        int x = i / n;
        int y = i % n;
        matrix[x][y] = disvalue(gen);
    }
    // 生成障碍物位置
    for(int i =0; i<numObstacles; i++){
        // 随机选择行Row和列Col
        std::uniform_int_distribution<> disRow(0, m-1);  
        std::uniform_int_distribution<> disCol(0, n-1);
        int row;
        int col;
        do{
             row = disRow(gen);
             col = disCol(gen);
        }while(!matrix[row][col]);   // 确保位置之前没有障碍物，未被占用, matrix[row][col]为1才跳出循环
        matrix[row][col] = 0;       // 生成障碍物位置，并起到标记作用
    }

    // 打印矩阵( 注意：字符串和数字不可以 一起用三目运算符)
    for (const auto &row : matrix) {
        for (const auto &elem : row) {
            std::cout << elem << " ";  
        }
        std::cout << std::endl;
    }
    return matrix;
}

// 检查坐标是否有效且未访问过
bool isValid(int Point_Row, int Point_Col, int  m, int n,
            std::vector<std::vector<bool>>& visited,
            std::vector<std::vector<int>>& obstacle_matrix ){
        if( Point_Row <m && Point_Row >=0 && Point_Col <n && Point_Col >=0
            && obstacle_matrix[Point_Row][Point_Col] != 0 && !visited[Point_Row][Point_Col]){
            return true;
        }else{
            return false;
        }

}

// 3. 实现判断输入起点终点能否联通 用深度优先搜索和广度优先搜索分别完成

    // 实现判断输入起点终点能否联通 用深度优先搜索完成
    /*
    深度优先搜索算法的策略步骤如下：
    1. 直接用递归遍历来做 或者 选择栈容器来存储数据 
    2. 放入一个起始节点，起始节点有4个搜索方向，上下左右
    3. 进入搜索的循环：
      （1）根据事先给定的规则，先弹出一个节点（这里的规则是 “最深层级的”），并将该节点加入close list容器
      （2）判断弹出的这个节点是不是目标点
      （3）扩展弹出节点的所有未被访问过的所有邻居
      （4）将发现的新的节点加入到栈容器中
      （5）持续到栈容器中没有节点可被弹出了， 或者 到达了目标点
    */
bool DFS_Connect(int startRow, int startCol,int endRow, int endCol,
                std::vector<std::vector<bool>>& visited,std::vector<std::vector<int>>& obstacle_matrix){

   
    // // 获取matrix的行数,列数
    // int matrix_rowcount = obstacle_matrix.size();
    // int matrix_colcount = obstacle_matrix.empty() ? 0 :obstacle_matrix[0].size();

    // 检查输入的起点，终点坐标是否有效 以及 检查是否到达终点
    // 检查起，终点是否障碍物点，是，则不联通
    if( obstacle_matrix[startRow][startCol] == 0 || obstacle_matrix[endRow][endCol] == 0 ){
        std::cerr << " start or end position are on the obstacle." << std::endl;
        return false;
    }
    if( startRow == endRow && startCol == endCol){     //  检查是否到达终点
        std::cout << " both positions are  not the obstacle ." << std::endl;
        std::cout << " DFS:  Reach the end "<< std::endl; 
        return true;
    }

   // 标记当前位置为已访问
   visited[startRow][startCol] = true;

   // 定义可移动的四个方向 向上（x坐标不变（0），y坐标加1）,向右，向下，向左, 右上，左上，右下，左下
   int directions[8][2] = { {0,1},{1,0},{0,-1},{-1,0},{1,1},{-1,1},{1,-1},{-1,-1}};
   for(int i = 0; i<8;i++){
    int newPoint_Row = startRow + directions[i][0];
    int newPoint_Col = startCol + directions[i][1];
    if(isValid(newPoint_Row, newPoint_Col, obstacle_matrix.size(), obstacle_matrix[0].size(), visited, obstacle_matrix)){
        if(DFS_Connect(newPoint_Row,newPoint_Col,endRow, endCol,visited, obstacle_matrix)){
            return true;
        }
    }
   }
   
   return false;
}

   // 用广度优先搜索完成
       /*
    广度优先搜索算法的策略步骤如下：
    1. 选择队列容器来存储数据 
    2. 放入一个起始节点，起始节点有4个搜索方向，上下左右
    3. 进入搜索的循环：
      （1）根据事先给定的规则，先弹出一个节点（这里的规则是 “最浅层级的”），并将该节点加入close list容器
      （2）判断弹出的这个节点是不是目标点
      （3）扩展弹出节点的所有未被访问过的所有邻居
      （4）将发现的新的节点加入到栈容器中
      （5）持续到栈容器中没有节点可被弹出了， 或者 到达了目标点
    */
bool BFS_Connect(int startRow, int startCol,int endRow, int endCol,
                std::vector<std::vector<bool>>& visited,std::vector<std::vector<int>>& obstacle_matrix){
    
    // 获取matrix的行数,列数
    int matrix_rowcount = obstacle_matrix.size();
    int matrix_colcount = obstacle_matrix.empty() ? 0 :obstacle_matrix[0].size();

    // 检查输入的起点，终点坐标是否有效 以及 检查是否到达终点
    // 检查起，终点是否障碍物点，是，则不联通
    if( startRow >= matrix_rowcount || startRow <0 || startCol >=  matrix_colcount || startCol <0
        || obstacle_matrix[startRow][startCol] ==0 || obstacle_matrix[endRow][endCol] ==0 ){
        std::cerr << " start or end position are on the obstacle or unreasonable " << std::endl;
        return false;
    }

    // 创建队列来保存数据
    std::queue< std::pair<int,int> > q;
    q.push({startRow,startCol});

    // 标记当前位置为已访问
    visited[startRow][startCol] = true;

    while(!q.empty()){

        auto front = q.front();
        q.pop();
        int row_x = front.first;
        int col_y = front.second;

        if(row_x == endRow && col_y == endCol){    //  检查是否到达终点
            std::cout<< " BFS: Connect and Reach the end " << std::endl;
            return true;
        }

        // 定义可移动的八个方向 向下（x坐标不变（0），y坐标加1）,向右，向上，向左
        int directions[8][2] = { {0,1},{1,0},{0,-1},{-1,0},{1,1},{-1,1},{1,-1},{-1,-1}};
        for(int i = 0; i< 8; i++){

            int new_x = row_x + directions[i][0];
            int new_y = col_y + directions[i][1];
            if(isValid(new_x, new_y, obstacle_matrix.size(), obstacle_matrix[0].size(),
                    visited, obstacle_matrix )){
                if(!visited[new_x][new_y]){  // 判断这个新的节点有无访问过
                    q.push( {new_x, new_y} );
                    visited[new_x][new_y] = true;
                }

            }
        }
    }
    return false;
}

// 4. 用dijkstra找到最短路径
    /*
    dijkstra算法的策略步骤如下：
    1.定义重要的容器：
        1）. 选择优先级队列容器来存储数据 
        2）. 选择距离向量minDists， 存储该节点到起始节点的最短距离
        3）. 定义prev向量， 记录最短路径的前驱节点
        4）. 定义输出的最短路径的向量 path
        5) . 增加边的权重（随机增加）的容器（在矩阵初始化中已做处理）
    2. 放入一个起始节点，起始节点有4个搜索方向，上下左右
    3. 进入搜索的循环：
      （1）g(n)是起点到当前节点n的累计代价，弹出最小的g(n)的节点，并将该节点加入close list容器（ visited 容器）
      （2）判断弹出的这个节点是不是目标点
      （3）扩展弹出节点的所有未被访问过的所有邻居，，更新g(m)值   将发现的新的节点加入到优先队列中
            ·若g(m)之前为无穷大，则用g(m) = g(n) +Cnm
            .若g(m)有值，即已被其他节点发现过一次，m已在优先级队列中，则比较当前g(m) 和 g(n) + Cnm ，保留最小的g值  （不用加入队列）
      （4）持续到栈容器中没有节点可被弹出了， 或者 到达了目标点
    */

   // 参数：搜索起点，搜索终点，访问的记录容器，带障碍物的随机权重矩阵

void Djt_shortest_path(int startRow, int startCol,int endRow, int endCol,
                std::vector<std::vector<bool>>& visited,std::vector<std::vector<int>>& obstacle_matrix){

    // pq队列存三个模版参数，第一个是队列元素类型(外层int用于存储节点的优先级或权重，内层std::pair<int, int>存储节点的位置或坐标)
          // 第二个是存储优先队列元素的底层容器类型，第三个是优先队列的比较函数对象,较小的优先级高，排前面
    std::priority_queue< std::pair< int, std::pair<int,int> >, 
        std::vector< std::pair< int, std::pair<int,int>> >,
        std::greater< std::pair< int, std::pair<int,int>> > > pq;

    int m = obstacle_matrix.size();    // m行
    int n = obstacle_matrix[0].size(); // n列
    
    // 判断起点坐标 合理性
    if( startRow >= m || startRow <0 || startCol >=  n || startCol <0
        || obstacle_matrix[startRow][startCol] == 0 || obstacle_matrix[endRow][endCol] == 0){
        std::cout << " Djt: start or end position are on the obstacle or unreasonable " << std::endl;
        return;
    }

    // 初始化距离向量 minDist，参数1:网格总单元格数，初始化所有值为最大整数（即未访问）  
    // 用于存储起点到当前节点的最短距离   
    std::vector<int> minDists( m * n, INT_MAX);

    // 前驱节点的一维向量 存的是坐标 prev  记录最短路径的前驱节点。初始化所有值为 {-1, -1}，表示没有前驱节点
    std::vector<std::pair<int,int>> prev( m * n, { -1,-1});

    // 最短距离的路径的向量容器 存的是坐标 path
    std::vector<std::pair<int, int>> path;

    pq.push({0, {startRow,startCol}}); // 将起点和初始距离（0）推入优先队列


    // 设置起点到自身的距离为0
    minDists[startRow * n + startCol] = 0;  
    
    // 标记当前位置为已访问
    visited[startRow][startCol] = true;

    // 当优先队列不为空时，循环继续
    while(!pq.empty()){
       
        // 从优先队列中获取当前距离最短的节点及其距离和坐标
        int curDist = pq.top().first;
        int curX = pq.top().second.first;
        int curY = pq.top().second.second;

        pq.pop(); // 弹出节点

        if(curX == endRow && curY == endCol){    //  检查是否到达终点
            std::cout<< " Djt_shortest_path: Connect and Reach the end " << std::endl;

            // 打印搜素后，该点到起点的距离值
            std::cout<< " Djt: 打印该点到起点的距离值(无穷大则代表未访问到该点，或者为障碍物点) "<< std::endl;
            for (int i = 0; i < m ; i++) {
                for (int j = 0; j < n ; j++) {
                    std::cout << minDists[i * n + j] << " ";
                }
                std::cout << std::endl;
            }
                // 输出最短路径
            path.push_back({endRow, endCol});
            // prev[i].first, prev[i].second 是当前i节点存储的 前驱节点的位置信息(X,Y)
            for (int i = endRow * n + endCol; i != startRow * n + startCol; i = prev[i].first * n + prev[i].second) {
                path.push_back({prev[i].first, prev[i].second});
            }

            // std::reverse(path.begin(), path.end());  // 直接在翻转 path
            // rbegin() 函数 , 可以获取 正向迭代器 的 尾部元素迭代器
            for (auto it = path.rbegin(); it != path.rend(); ++it) {
                // std::cout << " 找到最短路径 " << std::endl;
                std::cout << "(" << it->first << ", " << it->second << ") " << "-->";
                //std::cout << " --> " << std::endl;
            }
            std::cout << " 最短路径长度为： "<< minDists[endRow * n + endCol]<< std::endl;
            return;
        }

        // 如果当前节点的已知距离大于记录的最短距离，则跳过此次迭代
        if( curDist > minDists[curX * n + curY] ){
            continue;
        }

        int directions[8][2] = { {0,1},{1,0},{0,-1},{-1,0},{1,1},{-1,1},{1,-1},{-1,-1} };  // 定义八个方向

        for( auto& dir:directions){
            // 根据当前方向计算新节点的坐标
            int newX = curX + dir[0];
            int newY = curY + dir[1];
            // 检查新节点的坐标是否有效（在网格内且不是障碍物），且是否被访问过
            if(isValid(newX, newY, m, n, visited, obstacle_matrix)){
            // 计算通过当前边到达新节点的总距离
                int newDist = curDist + obstacle_matrix[newX][newY];
                visited[newX][newY] = true;
            // 如果新计算的距离小于已知的最短距离，则更新最短距离
                if( newDist < minDists[newX * n + newY] ){
                    // 更新 minDist 并将新节点及其新距离推入优先队列
                    minDists[newX * n + newY] = newDist;
                    prev[newX * n + newY] = {curX, curY}; // 记录前一个节点的坐标（记录最小距离时，上一个节点信息）
                    pq.push( { newDist, {newX,newY} });
                }
            }
        }
    }
    std::cout<< " Djt_shortest_path: Not Connect " << std::endl;
} 

// 欧几里得距离
int EucliDistance(int curX, int curY,int endRow, int endCol) {
    return sqrt(( curX - endRow)* ( curX - endRow) + (curY - endCol)*(curY - endCol));
}

// 5. 用A*找到最短路径
// A* 在Dijkstra的基础上，加了启发式函数的代价（从当前节点到最终节点的估计代价）
void Astar_shortest_path(int startRow, int startCol,int endRow, int endCol,
                std::vector<std::vector<bool>>& visited,std::vector<std::vector<int>>& obstacle_matrix){

    int m = obstacle_matrix.size();
    int n = obstacle_matrix[0].size();
    // 优先级队列 存储数据
    // std::pair<int , std::pair<int,int>参数含义：起始节点到当前节点的距离 当前节点的坐标
    std::priority_queue<std::pair<int , std::pair<int,int> >,
    std::vector<std::pair<int , std::pair<int,int>> > , 
    std::greater<std::pair<int , std::pair<int,int> >> > pq;  // 注意第一个参数不是std::vector<int , std::pair<int,int>

    // 加入h(n)的代价,初始化代价为0
    // std::vector<std::vector<int>> heurist_matrix(m,  std::vector<int>(n,0) );
    // f(n)的代价 
    // std::vector<std::vector<int>> totalcost_matrix = obstacle_matrix;

    // 距离矩阵 minDists 存的是单个值 即 g(n) 参数 记录的是 当前节点距离起始节点的距离
    std::vector< std::vector<int> > minDists(m, std::vector<int> (n,INT_MAX) );

    std::vector< std::vector<int> > total_minDists(m, std::vector<int> (n,INT_MAX) ); //加上了估计代价值
    // 前驱节点的一维向量 存的是坐标 prev
    std::vector<std::pair<int , int>> prev(m*n, {-1,-1});

    // 最短距离的路径的一维 存的是坐标 path
    std::vector<std::pair<int , int>> path;   // 赋值的形式，要熟悉

    // 判断起点合理性
    if(startRow > m || startRow <0 || startCol> n || startCol<0 || obstacle_matrix[startRow][startCol] ==0 ){
        std::cout << " 不合理 " <<std::endl;
        return;
    }

    minDists[startRow][startCol] = 0;  // 别漏掉这个  赋值的形式

    total_minDists[startRow][startCol] = EucliDistance(startRow, startCol, endRow, endCol) ;

    // 初始化起始队列
    pq.push({ total_minDists[startRow][startCol], {startRow,startCol} });  // 赋值的形式

    // visited[startRow][startCol] = true; //标记已访问

    while(!pq.empty()){

        int curDist = pq.top().first;
        int curX = pq.top().second.first;
        int curY = pq.top().second.second;

        visited[curX][curY] = true; //标记已访问
        pq.pop(); // 弹出
         
        if(curDist> total_minDists[curX][curY]){ 
            continue;
        }
        // 到达最终的节点
        if(curX == endRow && curY == endCol ){ 
            std::cout << " Astar success " << std::endl;
            // 打印矩阵的信息
            for(int i = 0; i <m; ++i){
                for(int j =0 ; j<n; ++j){
                    std::cout<< minDists[i][j]<< "   ";
                }
                std::cout<<std::endl;
            }

            // path 赋值
            path.push_back({endRow,endCol});
            for( int i = endRow *n +endCol; i!= startRow*n +startCol; i= prev[i].first *n +prev[i].second){
                path.push_back({prev[i].first,prev[i].second});
            }
            //打印最短路径的信息
            for(auto it = path.rbegin(); it!= path.rend(); it++){
                std::cout << " ( " << it->first <<" , " << it->second << " ) " << " -->" ;
                
            }
            std::cout << " Astar最短路径为: " << minDists[endRow][endCol]<<std::endl;
            return;
        }
        // 定义搜索的方向
        int directions[8][2] = { {0,1},{1,0},{0,-1},{-1,0},{1,1},{-1,1},{1,-1},{-1,-1} };
        for(auto& dir:directions){

            int newX = curX + dir[0];
            int newY = curY + dir[1];
            // int newDist = curDist + obstacle_matrix[newX][newY];  // minDists[newX][newY] 不能放这里，可能这个节点无效的
            // visited[newX][newY] = true;
            // 判断该节点合理性
            if( isValid(newX, newY, obstacle_matrix.size(), obstacle_matrix[0].size(),
                    visited, obstacle_matrix ) ){
                int newDist = minDists[curX][curY] + obstacle_matrix[newX][newY];  // newDist为g(n)的值
                int total_newDist = newDist + EucliDistance(newX, newY, endRow, endCol);
                visited[newX][newY] = true;
                if( total_newDist< total_minDists[newX][newY] ){  // total_newDist小于 之前记录的距离 更新

                    minDists[newX][newY] = newDist;
                    total_minDists[newX][newY] = total_newDist;
                    pq.push({ total_newDist, {newX,newY} });
                    prev[ newX*n +newY ] = {curX,curY};
                }
            }
        }
    }
    std::cout<< " Astar_shortest_path: Not Connect " << std::endl;

}

// 6. 用a＊考虑车辆碰撞 1. 构建车辆的box  2. 障碍物的碰撞检测（点与box，线与box，box和box，可先做点，线）

// 先基于当前节点车辆的box，车辆有倾斜方向，也就是点有theta方向

struct VehicleNode {
    int x, y;                  // 节点的坐标
    // double G, H, F;            // A*算法中的成本值
    //  VehicleNode *parent;       // 指向父节点的指针
    bool direction;            // 车辆方向（前进，后退）
    double length;             // 车辆的长度
    double width;              // 车辆的宽度
    double turningRadius;      // 车辆的最小转弯半径
    double angle;              // 车辆的角度（弧度）
    // double speed;              // 车辆速度
    // double maxSpeed;           // 车辆的最大速度
    // double acceleration;       // 车辆的加速度


    // 构造函数，用于初始化节点和车辆参数
    VehicleNode(int x, int y, bool direction, double len, double wid, double turnRad,double angle)
        : x(x), y(y), direction(direction), 
          length(len), width(wid), turningRadius(turnRad), angle(angle) {}
};

// 障碍物的碰撞检测（点与box，线与box，box和box，可先做点，线）

bool Astar_isCollisionFree(const std::vector<std::vector<int>>& matrix, const VehicleNode& vehicle_node) {
    // 根据车辆的速度、方向和动态约束进行碰撞检测
    return true;
}

// 在A*中加入 障碍物的碰撞检测，要考虑到车辆的box，用点和线障碍物去做

// 7. 混合a＊ （考虑车辆运动学等）

// 1. 实现一个随机生成m*n矩阵的程序
// 2. 实现n个随机位置生成障碍物
// 3. 实现判断输入起点终点能否联通 用深度优先搜索和广度优先搜索分别完成
// 4. 用dijkstra找到最短路径
// 5. 用A*找到最短路径
// 6. 用a＊考虑车辆碰撞
// 7. 混合a＊


int main(){

    // std::vector<std::vector<int>> matrix = generateMatrix();

    // 2. 实现n个随机位置生成障碍物
    // 输入矩阵的大小，m行，n列
    int matrix_row, matrix_col;
    std::cout << " Enter the number of rows (m) ";
    std::cin >> matrix_row;
    std::cout << " Enter the number of columns (n) ";
    std::cin >> matrix_col;
    std::vector<std::vector<int>> obstacle_matrix = generate_obstacle_Matrix(matrix_row, matrix_col);

    // 3. 实现判断输入起点终点能否联通 用深度优先搜索和广度优先搜索分别完成
    int startRow, startCol;
    int endRow, endCol;
    std::cout << " Enter the start position (Row Col) ";
    std::cin  >> startRow >> startCol;
    std::cout << " Enter the end position (Row Col) ";
    std::cin  >> endRow >> endCol;

    // visited 会被初始化为 false，表示所有节点都未被访问
    std::vector<std::vector<bool>> visited(matrix_row, std::vector<bool>(matrix_col, false));
    if( DFS_Connect(startRow, startCol,endRow,endCol,visited,obstacle_matrix) ){
        std::cout<< " DFS: connected "     << std::endl;
    }else{
        std::cout<< " DFS: not connected " << std::endl;
    }
    
    // 重新初始化visited
    // visited.resize(matrix_row, std::vector<bool>(matrix_col, false));
    visited = std::vector<std::vector<bool>>(matrix_row, std::vector<bool>(matrix_col, false));
    if( BFS_Connect(startRow, startCol,endRow,endCol,visited,obstacle_matrix) ){
        std::cout<< " BFS: connected "     << std::endl;
    }else{
        std::cout<< " BFS: not connected " << std::endl;
    }

    // 重新初始化visited
    visited = std::vector<std::vector<bool>>(matrix_row, std::vector<bool>(matrix_col, false));
     //  Djt_shortest_path( startRow, startCol, endRow, endCol, visited, obstacle_matrix );
    MEASURE_TIME(Djt_shortest_path, startRow, startCol, endRow, endCol, visited, obstacle_matrix);

    // 重新初始化visited
    visited = std::vector<std::vector<bool>>(matrix_row, std::vector<bool>(matrix_col, false));
    // Astar_shortest_path( startRow, startCol, endRow, endCol, visited, obstacle_matrix );
    MEASURE_TIME(Astar_shortest_path, startRow, startCol, endRow, endCol, visited, obstacle_matrix);

   //  VehicleNode VehicleNode = {};

    return 0;
}