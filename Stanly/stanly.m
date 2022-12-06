clc;
clear;
load('x.mat');
load('y.mat');

% @author:DaQiao
% @time:2022/11/30
% @brief stanly算法是利用前轮的横向误差和航向误差来实现对轨迹的跟踪。
% input: 规划轨迹ref(x,y,theta)，当前航向角heading,GPS位置信号pos(x,y)
% param: 车辆参数轴距L，预瞄距离Ld=kv*v+L0,控制时间dt，纵向控制速度增益kp和目标速度targetspeed
% output:前轮转角delta(通过方向盘与前轮转角ratio，可以计算出方向盘转角)
% 算法流程：1. 搜索最邻近点，搜索预瞄点;
%           2. 计算横向误差和航向误差;
%           3. 计算前轮转角
%% 1.参数定义
kv=0.01;
kp=0.8;
L0=2;
dt=0.1;
L=2.9;
%ref(:,1)=plan_x';
%ref(:,2)=plan_y';
ref(:,1)=linspace(0,10,100)';
ref(:,2)=ref(:,1).^2;
targetSpeed=10;

pos0=ref(1,:);
heading0=0 ;
v0=0;

pos=pos0;
heading=heading0;
v=v0;
targetIndex=0;

posAct=pos;