clear
% 参数
alpha0=0;
a0=0;
alpha=[ pi/2 0 -pi/2 pi/2 -pi/2];
a=[  0 0.345 0 0 0];
d=[0 0 0.09 0.295 0 0];
% test=[-0.25*pi 0.49*pi -1.1*pi 0.9*pi 0.2*pi 0];
test=[0.8*pi 0.8*pi -pi 0 0 0];
L(1) = Link('alpha', alpha0,         'a', a0,    'd', d(1),  'modified');
L(1).qlim=[-pi,pi];
L(2) = Link('alpha', alpha(1),      'a', a(1), 'd', d(2),  'modified');
L(2).qlim=[15/180*pi,140/180*pi];
L(3) = Link('alpha', alpha(2),         'a', a(2),'d', d(3),  'modified');
% L(3).qlim=[-250/180*pi,-45/180*pi];%-1.38  --  -0.25
L(3).qlim=[-250/180*pi,-0.5*pi];%-1.38  --  -0.25
L(4) = Link('alpha', alpha(3),     'a', a(3),    'd', d(4),  'modified');
L(4).qlim=[-pi,pi];
L(5) = Link('alpha',alpha(4),      'a', a(4),    'd', d(5),  'modified');
L(5).qlim=[-100/180*pi,80/180*pi];
L(6) = Link('alpha', alpha(5),     'a', a(5),    'd', d(6),  'modified');
L(6).qlim = [-2*pi,2*pi];
robot0 = SerialLink(L,'name','engineer');

% 可视化初始配置
figure(1);
robot0.plot(test);
title('机械臂初始配置');

%% 蒙特卡洛法计算机械臂工作空间
numSamples = 30000; % 采样点数
points = zeros(numSamples, 3); % 存储末端位置

% 遍历所有关节，随机采样
for i = 1:numSamples
    % 生成随机关节角度（在限制范围内）
    q = zeros(1,6);
    q(1) = L(1).qlim(1) + (L(1).qlim(2) - L(1).qlim(1)) * rand();
    q(2) = L(2).qlim(1) + (L(2).qlim(2) - L(2).qlim(1)) * rand();
    q(3) = L(3).qlim(1) + (L(3).qlim(2) - L(3).qlim(1)) * rand();
    q(4) = L(4).qlim(1) + (L(4).qlim(2) - L(4).qlim(1)) * rand();
    q(5) = L(5).qlim(1) + (L(5).qlim(2) - L(5).qlim(1)) * rand();
    q(6) = L(6).qlim(1) + (L(6).qlim(2) - L(6).qlim(1)) * rand();
    
    % 计算机械臂末端位置（假设末端是第6个连杆）
    T = robot0.fkine(q);
    points(i,:) = T.t(1:3)'; % 提取x,y,z坐标
end

% 可视化工作空间
figure(2);
scatter3(points(:,1), points(:,2), points(:,3), 5, 'filled', 'MarkerFaceAlpha', 0.3);
% title('机械臂工作空间（蒙特卡洛采样）');
% xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
% grid on;
% axis equal;
hold on;

% 提取点云边界
shrinkFactor = 0.5; % 收缩因子，调整边界紧密度（0~1，越小边界越紧）
k = boundary(points, shrinkFactor);

% 绘制边界
trisurf(k, points(:,1), points(:,2), points(:,3), 'FaceColor', 'red', 'FaceAlpha', 0.1, 'EdgeColor', 'none');
title('机械臂工作空间（蒙特卡洛采样及边界）');
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
grid on;
axis equal;
hold off;