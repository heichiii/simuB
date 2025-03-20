clear
%参数
alpha0=0;
a0=0;
alpha=[ pi/2 0 -pi/2 pi/2 -pi/2];
a=[  0 0.345 0 0 0];
d=[0 0 -0.09 0.295 0 0];
L(1) = Link('alpha', alpha0,         'a', a0,    'd', d(1),  'modified');
L(1).qlim=[-pi,pi];
L(2) = Link('alpha', alpha(1),      'a', a(1), 'd', d(2),  'modified');
L(2).qlim=[0,pi];
% L(2).qlim = [-2*pi,2*pi];
L(3) = Link('alpha', alpha(2),         'a', a(2),'d', d(3),  'modified');
% L(3).qlim=[-1.5*pi,-0.5*pi];
L(3).qlim = [-2*pi,2*pi];
L(4) = Link('alpha', alpha(3),     'a', a(3),    'd', d(4),  'modified');
L(4).qlim=[-pi,pi];
L(5) = Link('alpha',alpha(4),      'a', a(4),    'd', d(5),  'modified');
L(5).qlim=[-pi,pi];
L(6) = Link('alpha', alpha(5),     'a', a(5),    'd', d(6),  'modified');
L(6).qlim = [-2*pi,2*pi];

%符号量
syms th1 th2 th3 th4 th5 th6 x y z yaw pitch roll r
s1=sin(th1);
c1=cos(th1);
s2=sin(th2);
c2=cos(th2);
s3=sin(th3);
c3=cos(th3);
s4=sin(th4);
c4=cos(th4);
s5=sin(th5);
c5=cos(th5);
s6=sin(th6);
c6=cos(th6);
ca0=1;
sa0=0;
ca1=0;
sa1=1;
ca2=1;
sa2=0;
ca3=0;
sa3=-1;
ca4=0;
sa4=1;
ca5=0;
sa5=-1;

% R36_sol=[0 0 1;
%         0 1 0;
%         1 0 0];
% R36_sol=[0.7071 0 -0.7071;
%         0 1 0;
%         0.7071 0 0.7071]
T01=[c1 -s1 0 a0;
    s1*ca0 c1*ca0 -sa0 -sa0*d(1);
    s1*sa0 c1*sin(alpha0) ca0 ca0*d(1);
    0 0 0 1];
T12=[c2 -s2 0 a(1);
    s2*ca1 c2*ca1 -sa1 -sa1*d(2);
    s2*sa1 c2*sa1 ca1 ca1*d(2);
    0 0 0 1];
T23=[c3 -s3 0 a(2);
    s3*ca2 c3*ca2 -sa2 -sa2*d(3);
    s3*sa2 c3*sa2 ca2 ca2*d(3);
    0 0 0 1];
T34=[c4 -s4 0 a(3);
    s4*ca3 c4*ca3 -sa3 -sa3*d(4);
    s4*sa3 c4*sa3 ca3 ca3*d(4);
    0 0 0 1];
T45=[c5 -s5 0 a(4);
    s5*ca4 c5*ca4 -sa4 -sa4*d(5);
    s5*sa4 c5*sa4 ca4 ca4*d(5);
    0 0 0 1];
T56=[c6 -s6 0 a(5);
    s6*ca5 c6*ca5 -sa5 -sa5*d(6);
    s6*sa5 c6*sa5 ca5 ca5*d(6);
    0 0 0 1];

theta1=0;
theta2=pi/2;
theta3=-pi;
% *[1 0 0;0 0 1;0 -1 0]

% solve theta4 5 6
yaw_in=35.3;
pitch_in=-30;
roll_in=54.7;
yaw_in_r=yaw_in/180*pi;
pitch_in_r=pitch_in/180*pi;
roll_in_r=roll_in/180*pi;
R06 =[  cos(pitch_in_r)*cos(yaw_in_r),                                              -cos(pitch_in_r)*sin(yaw_in_r),                 sin(pitch_in_r);
    cos(roll_in_r)*sin(yaw_in_r) + cos(yaw_in_r)*sin(pitch_in_r)*sin(roll_in_r), cos(roll_in_r)*cos(yaw_in_r) - sin(pitch_in_r)*sin(roll_in_r)*sin(yaw_in_r), -cos(pitch_in_r)*sin(roll_in_r);
    sin(roll_in_r)*sin(yaw_in_r) - cos(roll_in_r)*cos(yaw_in_r)*sin(pitch_in_r), cos(yaw_in_r)*sin(roll_in_r) + cos(roll_in_r)*sin(pitch_in_r)*sin(yaw_in_r),  cos(pitch_in_r)*cos(roll_in_r)]
    

T01_sol=subs(T01,{th1},{theta1});
T12_sol=subs(T12,{th2},{theta2});
T23_sol=subs(T23,{th3},{theta3});
R01_sol=T01_sol(1:3,1:3);
R12_sol=T12_sol(1:3,1:3);
R23_sol=T23_sol(1:3,1:3);
% 
% R03_sol=R01_sol*R12_sol*R23_sol;
% R36_sol=(R03_sol^-1)*R06;


T34_sol=subs(T34,{th4},{0});
R340_sol=T34_sol(1:3,1:3);
R040_sol=R01_sol*R12_sol*R23_sol*R340_sol;
R36_sol=R040_sol'*R06;

% R36_sol_v=vpa(R36_sol)

theta5=atan2(sqrt(R36_sol(3,1)^2+R36_sol(3,2)^2),R36_sol(3,3));
theta4=atan2(R36_sol(2,3)/sin(theta5),R36_sol(1,3)/sin(theta5));
theta6=atan2(R36_sol(3,2)/sin(theta5),-R36_sol(3,1)/sin(theta5));




% 显示机械臂（把上述连杆“串起来”）

% theta=double([theta1 theta2 theta3 theta4 theta5 theta6]);
theta=double([0 0 0 theta4 theta5 theta6]);


robot0 = SerialLink(L,'name','engineer');
robot0.teach();
title('六轴机械臂模型');
