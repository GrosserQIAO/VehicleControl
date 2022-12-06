clc;
clear;
load('x.mat');
load('y.mat');

% @author:DaQiao
% @time:2022/11/30
% @brief stanly�㷨������ǰ�ֵĺ������ͺ��������ʵ�ֶԹ켣�ĸ��١�
% input: �滮�켣ref(x,y,theta)����ǰ�����heading,GPSλ���ź�pos(x,y)
% param: �����������L��Ԥ�����Ld=kv*v+L0,����ʱ��dt����������ٶ�����kp��Ŀ���ٶ�targetspeed
% output:ǰ��ת��delta(ͨ����������ǰ��ת��ratio�����Լ����������ת��)
% �㷨���̣�1. �������ڽ��㣬����Ԥ���;
%           2. ����������ͺ������;
%           3. ����ǰ��ת��
%% 1.��������
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