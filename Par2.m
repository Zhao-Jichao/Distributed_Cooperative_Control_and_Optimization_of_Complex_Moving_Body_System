% 函数

clear

% 领航者状态：Q0
% 领航者速度：V0
% 领航者加速度：A0
% 跟随者估计位置：P_EsX、P_EsY


Q0 = [0 0]';
V0 = [0 0]';
A0 = [0 0]';

P_EsX_t0 = [pi/7*1, pi/7*2, pi/7*3, pi/7*4];
P_EsY_t0 = [pi/8*1, pi/8*2, pi/8*3, pi/8*4];


% 时间参数
tBegin = 0;
tEnd   = 10;
dT     = 0.1;


[T, P_EsX] = ode45(@Differ, [tBegin,tEnd], P_EsX_t0);
[TY,P_EsY] = ode45(@DifferY,[tBegin,tEnd], P_EsY_t0);

% 绘制结果图
figure(1)

subplot(2,1,1)
plot(T,P_EsX(:,1), T,P_EsX(:,2), T,P_EsX(:,3), T,P_EsX(:,4)); 
legend('运动体1','运动体2', '运动体3','运动体4'); title('机械臂运动轨迹'); grid on

subplot(2,1,2)
plot(TY,P_EsY(:,1), TY,P_EsY(:,2), TY,P_EsY(:,3), TY,P_EsY(:,4)); 
legend('运动体1','运动体2', '运动体3','运动体4'); title('机械臂运动轨迹'); grid on


% 微分方程
function P_EsX_Dot = Differ(t,P_EsX)
    % 系数
    alpha = [1.5 1.5 1.5 1.5]';
    % 跟随者之间关系矩阵
    L = [0  0  0  0;
        -1  1  0  0;
         0 -1  1  0;
         0  0 -1  1;];
    % 与领航者关系矩阵
    B = [1;
         0;
         0;
         0;];
     
    P_EsX_Dot = -alpha .* sign( L*P_EsX + B.*(P_EsX-sin(t)) );
end
% 微分方程
function P_EsY_Dot = DifferY(t,P_EsY)
    % 系数
    alpha = [1.5 1.5 1.5 1.5]';
    % 跟随者之间关系矩阵
    L = [0  0  0  0;
        -1  1  0  0;
         0 -1  1  0;
         0  0 -1  1;];
    % 与领航者关系矩阵
    B = [1;
         0;
         0;
         0;];
     
    P_EsY_Dot = -alpha .* sign( L*P_EsY + B.*(P_EsY+sin(t)) );
end




