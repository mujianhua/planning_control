# ROS Package
实现无人车规划与控制仿真及试验

Email: jianhua_mu@163.com

# Planning


# Control
主要控制算法为Model Predict Control(MPC)，其中用到了[OSQP](https://osqp.org/)求解器，并借鉴的百度[Apollo](https://github.com/ApolloAuto/apollo)的相关算法。

MPC中考虑侧倾动力学的公式
$$
m{\dot v_y} = {F_{yf}} - m{v_x}{\omega _z} - mg\sin ({\phi _t}) + {m_s}{h_{sr}}{\dot \omega _x}\\
{I_z}{\dot \omega _z} = ({l_f}{F_{yf}} - {l_r}{F_{yr}})\\
{\dot \phi _r} = {\omega _x}\\
{I_x}{\dot \omega _x} = {m_s}{h_{sr}}({\dot v_y} + {v_x}{\omega _z}) + {m_s}g{h_{sr}}\sin (\phi ) - {M_R}
$$

### csc_matrix
* m 行数
* n 列数
* 有效值个数
* 有效值数组
* 本列有效数字所在的行数
* 截止到本列截止的位置，共有多少位有效数字

[cpp读取yaml](https://blog.csdn.net/weixin_45024226/article/details/120279723)


# tools
绘制曲线

### plot_control
绘制控制参数曲线：转向、油门、刹车...

### plot_control_debug
调试跟踪控制器：包括轨迹曲线、侧向误差、航向角误差...


# bagfiles

保存数据并且回访使用

录制bag数据命令 ``rosbag record -o bag_name --duration=30 -a(/topic_name)``


# other

[链接自己的库](https://zhuanlan.zhihu.com/p/337377100)

**vscode 最新版本好像不再支持 Python2.7 ...**

显示DEBUG信息
```c++
(1)
rosconsole set <node> <logger> <level>
rosconsole set rosdebugtest ros.rosdebugtest debug

(2)
ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
                                ros::console::levels::Debug);
```

``std::shared_ptr``