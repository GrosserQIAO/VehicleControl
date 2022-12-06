% @author: DaQiao
% @time:2022/12/6
% @brief 实现无约束的MPC控制，这里假设预测时域和控制时域相同
%        对比dlqr，发现在N>4时，无约束的MPC相当于lqr算法


clc;
clear;

%连续空间方程
Ad=[1.1 2;0 0.95];
Bd=[0;0.079];
C=[1 0];
D=0;

%离散化
%dt=0.1;
%[Ad,Bd,Cd,Dd]=c2dm(A,B,C,D,dt);

%预测时域和控制时域
N=4;
x0=[1;1];
q=eye(2);
r=0.1;
X1=x0;
X2=x0;
xk1=x0;
xk2=x0;
k=dlqr(Ad,Bd,q,r);
for i=1:100
    u1=solverLinearMPC(Ad,Bd,xk1,q,r,N);
    u2=-k*xk2;
    xk1=Ad*xk1+Bd*u1;
    xk2=Ad*xk2+Bd*u2;
    X1=[X1,xk1];
    X2=[X2,xk2];
end
figure(1);
t=linspace(1,101,101);
plot(t,X1(1,:),'-');
hold on;
plot(t,X1(2,:),'-');

figure(2);
plot(t,X2(1,:),'-');
hold on;
plot(t,X2(2,:),'-');

function MPC_control=solverLinearMPC(Ad,Bd,xk,q,r,N)
    %F矩阵
    F=[];
    for i=1:N
        F=[F;Ad^i];
    end
    
    %Phi矩阵
    full=zeros(size(Bd));
    Phi=zeros(length(Bd(:,1))*N,length(Bd(1,:))*N);
    for i=1:N
        phi=[];
        for j=i-1:-1:0
            phi=[phi,Ad^j*Bd];
        end
        for k=i+1:N
            phi=[phi,full];
        end
        Phi(length(Bd(:,1))*(i-1)+1:length(Bd(:,1))*i,:)=phi;
    end
    
    %Q矩阵
    Q=[];
    for i=1:N
        Q=blkdiag(Q,q);
    end
    
    %R矩阵
    R=[];
    for i=1:N
        R=blkdiag(R,r);
    end
    
    %I矩阵
    I=eye(length(Bd(1,:)));
    for i=2:N
        I=[I,zeros(length(Bd(1,:)),length(Bd(1,:)))];
    end
    MPC_control=-I*inv(Phi'*Q*Phi+R)*Phi'*Q*F*xk;
end






