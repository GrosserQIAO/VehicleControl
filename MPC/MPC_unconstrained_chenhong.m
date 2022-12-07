% @author:  DaQiao
% @time:    2022/12/7
% @brief:   实现一种增量式无约束MPC控制，预测时域和控制时域可以改变；
%           可参考《模型预测控制》（陈虹）；
% @problem: 发现在该MPC模型仅对SISO系统可以实现无静差跟踪，但是对于MIMO会出现误差，而且改变R和Q，会
%           会使得输出值发散。目前我还没找到具体的原因是什么，并如何进行改正。

clear;
clc;
Ad=[1.1 2;0 0.95];
Bu=[0;0.079];
Bd=[0;0];
Cd=[1 0];
D=0;

Np=8;
Nc=8;
R=1;
Q=1;
dx=[0;0];
y0=0;
u0=0;
Y=y0;
U=u0;
Yd=sin(linspace(1,120,1200));
for i=1:1000
    Rk=[Yd(i+1);Yd(i+2);Yd(i+3);Yd(i+4);Yd(i+5);Yd(i+6);Yd(i+7);Yd(i+8)];
    du=solverMPC(Ad,Bd,Cd,Bu,Np,Nc,R,Q,dx,y0,0,Rk);
    u0=u0+du;
    
   %更新
    dx=Ad*dx+Bu*du;
    y0=Cd*dx+y0;
    
    Y=[Y,y0];
    U=[U,u0];
end
figure(1);
plot(linspace(1,1001,1001),Y,'-');
hold on;
plot(linspace(1,1200,1200),Yd,'-');
figure(2);
plot(linspace(1,1001,1001),U,'-');


function du=solverMPC(Ad,Bd,Cd,Bu,Np,Nc,R,Q,dx,y,dd,Rk)
    %Sx
    temp=Cd*Ad;
    Sx=temp;
    for i=2:Np
        temp=temp+Cd*Ad^i;
        Sx=[Sx;temp];
    end
    
    %I
    temp=eye(length(y(:,1)));
    I=temp;
    for i=2:Np
        I=[I;temp];
    end
    
    %Sd
    temp=Cd*Bd;
    Sd=temp;
    for i=2:Np
        temp=temp+Cd*Ad^(i-1)*Bd;
        Sd=[Sd;temp];
    end
    
    %Su
    full=zeros(size(Cd*Bu));
    Su=[];
    for i=1:Nc
        temp=0;
        su=[];
        for j=1:i
            temp=temp+Cd*Ad^(j-1)*Bu;
            su=[temp,su];
        end
        for k=i+1:Nc
            su=[su,full];
        end
        Su=[Su;su];
    end
    for i=Nc+1:Np
        su=su(:,1:end-size(Cd*Bu,1));
        temp=temp+Cd*Ad^(i-1)*Bu;
        su=[temp,su];
        Su=[Su;su];
    end
    
    %Gamay
    Gamay=[];
    for i=1:Np
        Gamay=blkdiag(Gamay,R);
    end
    
    %Gamau
    Gamau=[];
    for i=1:Nc
        Gamau=blkdiag(Gamau,Q);
    end
    
    %Iu
    Iu=eye(size(Q));
    I0=zeros(size(Q));
    for i=2:Nc
        Iu=[Iu,I0];
    end
    
    %计算Kmpc
    Kmpc=Iu*inv(Su'*(Gamay'*Gamay)*Su+Gamau'*Gamau)*Su'*(Gamay'*Gamay);
    
    %计算Ep
    Ep=Rk-Sx*dx-I*y-Sd*dd;
    du=Kmpc*Ep;
end
















