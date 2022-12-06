clc;
clear;
load('x.mat');
load('y.mat');

% @author:DaQiao
% @time:2022/11/28
% @brief 纯跟踪算法相当于一个P控制器：tan(delta)=(2L/ld^2 )*ed，ld=kv*v+l0;
% @brief 车辆参数：L轴距，v车速，dt采样时间，pos初始位置,heading航向角
% @brief 算法参数：L0预设预瞄距离，ref规划层位置，kv速度系数，kp比例系数
%  算法流程：1. 找到最邻近搜索点,最近预瞄点；
%           2. 计算误差；
%           3. 计算转向角；
%           4. 更新位置状态；

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

%% 
while targetIndex<length(ref)
    %找到最近参考点
    [targetpoint,targetIndex]=findLookeheadPoint(pos,ref,v,kv,L0);
    %计算误差e和输入值
    delta=purePursuitConthrol(targetpoint,pos,heading,kv,v,L0,L);
    a=kp*(targetSpeed-v)/dt;
    %更新位置
    [pos,heading,v]=updatestate(pos,heading,v,dt,a,L,delta);
    posAct(end+1,:)=pos;
end

plot(ref(:,1),ref(:,2),'-');
hold on;
plot(posAct(:,1),posAct(:,2),'r*');


%% 找到参考轨迹最近点
function [LookaheadPoint,index]=findLookeheadPoint(pos,ref,v,kv,L0)
    dis=zeros(1,length(ref));
    for i=1:length(ref)
        dis(i)=norm(ref(i,:)-pos);
    end
    [~,pt]=min(dis);
    
    % 找到ld最近点
    Ld=kv*v+L0;
    L=0;
    j=pt+1;
    while L<Ld && j<=length(ref)
        L=norm(ref(pt,:)-ref(j,:));
        j=j+1;
    end
    index=j-1;
    LookaheadPoint=ref(index,:);  
end

function delta=purePursuitConthrol(targetpoint,pos,heading,kv,v,L0,L)
    alpha=atan2(targetpoint(1,2)-pos(2),targetpoint(1,1)-pos(1))-heading;
    Ld=kv*v+L0;
    delta=atan2(2*L*sin(alpha),Ld);
end


function [posNew,headingNew,vNew]=updatestate(posOld,headingOld,vOld,dt,a,L,delta)
    posNew=posOld+vOld*dt*[cos(headingOld) sin(headingOld)];
    headingNew=headingOld+vOld*dt*tan(delta)/L;
    vNew=vOld+a*dt;
end












 