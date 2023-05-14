# Astar demo
还有很多细节问题没有完善，这只是一个简单的算法原理实现。

# Build


Clone the repository and catkin_make:


    git clone https://github.com/Rokiii1012/Astar_planner.git
    cd Astar_planner
    catkin_make
    
   
   
   # Run
   
         source devel/setup.bash
         roslaunch planner run_node.launch
         
     
     
     
     
 用rviz的插件选择起点和终点即可，规划出路径。想要第二次规划，需要重新启动（因为程序还没写好）。
