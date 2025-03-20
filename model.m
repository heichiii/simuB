clear
%参数
alpha=[0 pi/2 0 -pi/2 pi/2 -pi/2];
a=[0 0 0.345 0 0 0];
d=[0 0 -0.09 0.3 0.09 0];
theta=[0 pi/2 -pi 0 0 0];
syms theta1 theta2 theta3 theta4 theta5 theta6

L(1) = Link('alpha', alpha(1),         'a', a(1),    'd', d(1),  'modified');
L(1).qlim=[-pi,pi];
L(2) = Link('alpha', alpha(2),      'a', a(2), 'd', d(2),  'modified');
L(2).qlim=[0,pi];
L(3) = Link('alpha', alpha(3),         'a', a(3),'d', d(3),  'modified');
L(3).qlim=[-1.5*pi,0];
L(4) = Link('alpha', alpha(4),     'a', a(4),    'd', d(4),  'modified');
L(4).qlim=[-pi,pi];
L(5) = Link('alpha',alpha(5),      'a', a(5),    'd', d(5),  'modified');
L(5).qlim=[-pi,pi];
L(6) = Link('alpha', alpha(6),     'a', a(6),    'd', d(6),  'modified');
L(6).qlim = [-2*pi,2*pi];

% 显示机械臂（把上述连杆“串起来”）
robot = SerialLink(L,'name','engineer');
% robot.teach(theta);
% title('六轴机械臂模型');

% 正运动学计算末端执行器位姿
T = robot.fkine(theta);

% 数值方法求解逆运动学
q_sol = robot.ikine(T);
res=q_sol/pi*180

robot.teach(q_sol);
title('六轴机械臂模型');

% %% 各坐标系
% 
% 
% 
% %基准坐标系
% U=[1 0 0 0;
%    0 1 0 0;
%    0 0 1 0;
%    0 0 0 1];
% 
% %A:与基准坐标系重合
% R1=[1 0 0 0;
%    0 1 0 0;
%    0 0 1 0;
%    0 0 0 1];
% T1=[cos(theta1) -sin(theta1) 0 0;
%    sin(theta1) cos(theta1) 0 0;
%    0 0 1 0;
%    0 0 0 1];
% % A0=R1*U
% % A=R1*T1*U
% 
% %B:绕X转90度
% R2=[1 0 0 0;
%    0 0 -1 0;
%    0 1 0 0;
%    0 0 0 1];
% T2=[cos(theta2) -sin(theta2) 0 0;
%    sin(theta2) cos(theta2) 0 0;
%    0 0 1 0;
%    0 0 0 1];
% % B0=R2*A0
% % B=R2*T2*A
% 
% %C:x+0.345 y+0.09
% 
% R3=[1 0 0 0.345;
%     0 1 0 0;
%     0 0 1 -0.09;
%     0 0 0 1];
% T3=[cos(theta3) -sin(theta3) 0 0;
%    sin(theta3) cos(theta3) 0 0;
%    0 0 1 0;
%    0 0 0 1];
% % C0=R3*B0
% % C=R3*T3*B
% 
% %D:
% R4=[1 0 0 0;
%    0 0 1 0.3;
%    0 -1 0 0;
%    0 0 0 1];
% T4=[cos(theta4) -sin(theta4) 0 0;
%    sin(theta4) cos(theta4) 0 0;
%    0 0 1 0;
%    0 0 0 1];
% % D0=R4*C0
% % D=R4*T4*C
% 
% %E:
% R5=[1 0 0 0;
%    0 0 -1 -0.09;
%    0 1 0 0;
%    0 0 0 1];
% T5=[cos(theta5) -sin(theta5) 0 0;
%    sin(theta5) cos(theta5) 0 0;
%    0 0 1 0;
%    0 0 0 1];
% % E0=R5*D0
% % E=R5*T5*D
% 
% %F:
% R6=[1 0 0 0;
%    0 0 1 0;
%    0 -1 0 0;
%    0 0 0 1];
% T6=[cos(theta6) -sin(theta6) 0 0;
%    sin(theta6) cos(theta6) 0 0;
%    0 0 1 0;
%    0 0 0 1];
% %% 正解算
% F=R1*T1*R2*T2*R3*T3*R4*T4*R5*T5*R6*T6*U



% TT1=[cos(theta(1)) -sin(theta(1)) 0 0;
%    sin(theta(1)) cos(theta(1)) 0 0;
%    0 0 1 0;
%    0 0 0 1];
% TT2=[cos(theta(2)) -sin(theta(2)) 0 0;
%    sin(theta(2)) cos(theta(2)) 0 0;
%    0 0 1 0;
%    0 0 0 1];
% TT3=[cos(theta(3)) -sin(theta(3)) 0 0;
%    sin(theta(3)) cos(theta(3)) 0 0;
%    0 0 1 0;
%    0 0 0 1];
% TT4=[cos(theta(4)) -sin(theta(4)) 0 0;
%    sin(theta(4)) cos(theta(4)) 0 0;
%    0 0 1 0;
%    0 0 0 1];
% TT5=[cos(theta(5)) -sin(theta(5)) 0 0;
%    sin(theta(5)) cos(theta(5)) 0 0;
%    0 0 1 0;
%    0 0 0 1];
% TT6=[cos(theta(6)) -sin(theta(6)) 0 0;
%    sin(theta(6)) cos(theta(6)) 0 0;
%    0 0 1 0;
%    0 0 0 1];
% FF=R1*TT1*R2*TT2*R3*TT3*R4*TT4*R5*TT5*R6*TT6
