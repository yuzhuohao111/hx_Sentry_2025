火线战队 2025年哨兵导航学习包

首先致谢深圳北理莫斯科大学北极熊战队、四川大学火锅战队、吉林大学TARS_Go战队、湖南大学跃鹿战队的开源与交流。

0.首先了解接下来一年需要研究的东西:
        一个优秀的导航系统需要有一个良好的定位算法，基于稳定鲁棒的定位我们才能在上面进行路径搜索和轨迹优化以及运动控制 控制小车跟随全局路径的走，然后在运动的过程中需要一个感知周围的算法及地形分析算法来提取障碍物进行避障。
    所以 分为三个部分:
     1、定位 重定位  现在用的比较多定位的是lio算法 其中定位比较鲁棒的方案是基于先验地图进行初始位姿定位 然后基于这个先验地图在进行增量式建图来构建实时环境并且得到去畸变的点云来后续使用
     2、路径搜索 轨迹优化 运动控制 然后nav2包已经包含这部分所有需要用到的一些方案吧算是 足够用了 包含轨迹平滑、恢复行为 重规划等等
     3、环境感知  即地形分析包 主要是通过一系列手段得到障碍物信息来使用 也有很多开源 比如地面分析 cmu团队的地形分析 或者中科大提出来的那个
    
一、
    介绍：本项目基于nav2的导航框架做改进，双雷达融合做定位和避障，行为树进行自主决策, 分区域进行划分语义地图实现每个区域实现不同功能 从而实现过隧道的功能

    项目环境
    * Ubuntu22.04
    * ros2 humble

    ## 代码框架:
    .
    ├── rm_driver                       # Livox 驱动 修改适用双雷达将imu按照对应外参分别旋转
    ├── rm_nav_bringup                  # 启动文件
    ├── rm_localization                 # 里程计定位、重定位
        ├── fast-lio                        fastlio_localization 是icp初始位姿配准成功后给fastlio发标注位再进行定位  见501行 bool initial_pose_received = false;
        ├── point-lio                       发布odometry函数里面加上计算实际速度（见285-319行） 将去畸变点云cloud_registered_body 转换成世界坐标系（见231行 publish_frame_body函数）
        ├── point-lio-cxr                   发布odometry函数里面加上计算实际速度（见752-785行） 将去畸变点云cloud_registered_body 转换成世界坐标系（见619行）
        ├── icp_registration                icp重定位 将发布结果的z设成0（见262行）
        └── small_gicp_relocalization       gicp重定位
    ├── rm_navigation                   # nav2的插件包 将nav2里面一些包进行了修改以及添加新的功能包
        ├── nav2_navfn_planner              修改成八邻域搜索
        ├── nav2_planner                    
        ├── nav2_smac_planner               增加可视化 不太重要
        ├── pb_nav2_plugins                 恢复行为里面增加适应机器人坐标系的速度（适应不加虚拟坐标系下的） 然后修改搜索可通行区域不从机体中心搜索  206行  float r = 0.3f; 
        └── pb_omni_pid_pursuit_controller  pid控制器：增加自适应脱困 以及各个区域切不同的旋转pid转特定角度过隧道 并且有卡隧道多次进行换点 （初学者可以先读开源本来的代码 然后再结合我这个）
    ├── rm_perception                   # 感知功能包 内含点云处理、点云融合（其实就是将点云进行遍历）、imu工具包、地面分割、地形分析、将点云转为sacn进行slam建图  
        ├── cloud_process                   点云预处理包   按照区域滤除挡板点云
        ├── imu_complementary_filter        imu功能包    
        ├── linefit_ground_segementation    地面分割  加了一些函数 但是没用到最后  
        ├── merge_cloud_code                点云融合    
        ├── pointcloud_to_laserscan         点云转scan 建图用    
        └── terrain_analysis                地形分析    
    ├── rm_simulation                       # 仿真包以及实车xacro
    ├── auto_aim_interfaces                 # 视觉消息包
    ├── hx_decsion                          # 行为树
    └── rm_decision_interfaces              # 导航消息包

二、快速启动     

1. 安装依赖

    ```sh
    cd ~/hx_Sentry_2025

    rosdep install -r --from-paths src --ignore-src --rosdistro humble -y
    ```

2. 编译

    ```sh
    colcon build --symlink-install
    ```
    
3. 仿真

    ```sh
    ./sim_mapping,sh (建图)
    ./sim_nav.sh     (导航)
    ```
    实车

    ```sh
    ./real_mapping,sh (建图)
    ./real_nav.sh     (导航)
    ```
    记得切换对应的栅格地图
4. 地图保存

    ```sh
    保存栅格地图：ros2 run nav2_map_server map_saver_cli -f <YOUR_MAP_NAME>
    点云地图:ros2 service call /map_save std_srvs/srv/Trigger
    ```
5. 小工具

    ```sh
    ros2 run teleop_twist_keyboard teleop_twist_keyboard
    ```

0. 依赖
    1. 安装 [Livox SDK2](https://github.com/Livox-SDK/Livox-SDK2)

    ```sh
    git clone https://github.com/Livox-SDK/Livox-SDK2.git
    cd ./Livox-SDK2/
    mkdir build
    cd build
    cmake .. && make -j
    sudo make install
    ```
    2. 安装 [small_icp](https://github.com/koide3/small_gicp)

    ```zsh
    sudo apt install -y libeigen3-dev libomp-dev

    git clone https://github.com/koide3/small_gicp.git
    cd small_gicp
    mkdir build && cd build
    cmake .. -DCMAKE_BUILD_TYPE=Release && make -j
    sudo make install
    ```
    3 .安装深度相机库 （可选）
    
Tips:
    1. 保存点云 pcd 文件：需先在 [fast_lio/mid360.yaml](src/rm_localization/fast_lio/config/mid360.yaml) `pcd_save_en` 改为 `true`，并设置 .pcd 文件的路径，运行时新开终端输入命令 `ros2 service call /map_save std_srvs/srv/Trigger`，即可保存点云文件；或者直接终端ctrl+c 退出，即可保存点云文件。
    2. 保存地图：请参考 [如何保存 .pgm 和 .posegraph 地图？](https://gitee.com/SMBU-POLARBEAR/pb_rmsimulation/issues/I9427I)。地图名需要与 `YOUR_WORLD_NAME` 保持一致。
	3. 注意雷达驱动中雷达的相对位置 坐标变换 MID360_config.json
	4. 确保启动脚本后每个launch文件都启动成功
	5. 注意点云分割包中的参数 雷达距离地面的高度
	6. 注意雷达正方向  可以修改Pointlio中task
	7. 注意车的初始位置 可以修法Pointlio中的初始位置 （这个初始位置是基于原先地图上车的原来初始位置的坐标）
    8. 局部代价地图大小会影响局部路径的长度 即前瞻点距离 即拐弯时候跟随路径的效果

基于24代码优化点(注意事项)：
    1. 将navfn的a星修改成八邻域搜索

    2. 修改smac的混合a星 添加可视化 以及一些细节 更好的实现控制云台跟随路径方向（转弯半径为很小的时候就是一个全向模型）

    3. 修改恢复行为 将后退优化成后退到安全的区域 并且相应很快 并且改成速度发布于车体坐标系 适应适应底盘跟随模式 和适应巡航模式  并将r增大 使得每次触发不是在原点开始搜索安全空间

    4. 添加北极熊的pid控制器 更好的跟随全局路径 并且添加高曲率半径限制 修改旋转pid的输入值 可以控制云台朝向

	5. 增加Pointlio 基于先验地图进行定位 减小激光odometry的误差 ；并且增加卡尔曼滤波可以在一定误差内（xy坐标偏移不大 雷达与初始正方向的角度不到10度）都能初始化成功；并且提供旋转矩阵 使得雷达初始方向可以相对于原先设定的方向 yaw上旋转180

	6. 修改动态点云包 (不好用弃用) 采用华农的基于先验地图差分出来的包，经过调参 加上欧式聚类效果： 效果不好
								   加上点云裁剪效果：仿真：能提取台阶点云 但是上下坡点云杂乱
                                ***后续改进方案：将固定的一个z轴高度改成根据pcd的z轴高度来提取动态障碍物

	7. 修改地面分割的包 在原来基础上增加条件滤波 （记得修改参数）去除机体点云  台阶点云提取也比较准确，并且将狗洞的隔板滤除 保证斜坡也是可通行区域 更加符合地形分析 （滤波可选用） 
    
    8. 添加cmu的地形分析包 和配套的代价地图层 根据点云的i（强度）来判断是不是障碍物   但是参数不是很好调 后续优化

	9. 换上时空提素层 配备滤波滤除机体点云后 动态障碍物的提取效果不错 规划避障  

    10. 雷达驱动进行修改 准确将点云数据和imu数据都用对应外参进行旋转变换 适用双雷达

    11. 添加点云处理包 同步点云和odometry数据时间戳软同步 并对点云做处理 适用pointcloud2类型的 后续加上按照区域滤除点云了很好用有效去除挡板点云
    
    12. 添加点云融合包 将点云进行融合 并在发布pointcloud2类型的时候先进行处理再发布 可以避免地形分析的时候再进行处理
                                    发布custommsg类型的点云 做定位 但是待优化

    13. 用cloud-registered 或者cloud registered body这些去运动畸变的点云来避障 将body点云转到世界坐标系

    14. 将规划路径时加上区域判断过隧道的标志位 和决策的标志位共同抉择

    15. 修改pointlio 在发布odometry时给twist赋值 为机器人坐标系下实际的速度 然后再pid进行闭环 或者做脱困操作

    16. 修改局部代价地图大小 可以影响局部路径长度 或者前瞻点距离 可以在转弯处严格跟上路径
    
todo：
    1. 原始点云存在拖影晃动的现象 时间戳的问题？ 硬同步？ 待优化解决 点云融合加上去畸变处理点云? 
    2. 重定位算法 目前icp和slamtoolbox只能将初始点重定位 gicp只能过程中配准但是不好用
    3. 坐标系仍需优化       比如建图的时候不让slamtoolbox发布odom->map?将transform_publish_period改成0即可
    4. 控制避障 mppi 
    5. 避障还需优化 今年使用cmu的地形分析感觉怪怪的


狗洞调试：
       其实就是上位机控制云台或者底盘朝向沿着路径方向即可，如果可以直接控制底盘姿态的话是最优解，控制云台姿态就需要下位机写底盘跟随模式。笔者由于底盘没陀螺仪只能控制云台朝向，首先调了很久这个方法过洞 效果都不是很好速度特别慢才能跟上轨迹，于是卡地图bug 在过洞时直接把云台朝向转到正对着隧道方向即可这样可以保证速度可以提高 并且在洞里死了之后还能正常一个点回家。

最后再次致谢：
    深北莫北极熊战队的开源：https://github.com/SMBU-PolarBear-Robotics-Team/pb2025_sentry_nav.git
    四川大学火锅战队的开源：https://github.com/PolarisXQ/SCURM_SentryNavigation.git
    湖南大学跃鹿战队的开源：https://gitee.com/hnuyuelurm/hnunavigation_-ros2
    
结语：
回顾25赛季 调了三种类型的哨兵 无论是雷达固定构型还是大小yaw还是狗洞哨 从单雷达到雷达融合深度相机到双雷达融合 都发挥出应该的性能和水平 也都实现了应该实现的功能  在国赛的舞台上也能通过四个隧道 也算是没有遗憾了 希望此开源对后人有帮助
有任何代码问题请及时指出 qq：   2782489573  欢迎指点批评
