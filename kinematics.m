clear
%% 参数
alpha0=0;
a0=0;
alpha=[ pi/2 0 -pi/2 pi/2 -pi/2];
a=[  0 0.345 0 0 0];
d=[0 0 -0.09 0.295 0 0];
test=[0 pi/8 -pi pi/3 pi/6 pi/4];
% alpha0=0;
% a0=0;
% alpha=[ pi/2 0 -pi/2 pi/2 -pi/2];
% a=[  0 0.345 0.2 0 0];
% d=[0 0 -0.09 0.295 0 0];
%% 显示机械臂
L(1) = Link('alpha', alpha0,         'a', a0,    'd', d(1),  'modified');
L(1).qlim=[-pi,pi];
L(2) = Link('alpha', alpha(1),      'a', a(1), 'd', d(2),  'modified');
L(2).qlim=[0,pi];
L(3) = Link('alpha', alpha(2),         'a', a(2),'d', d(3),  'modified');
L(3).qlim=[-1.5*pi,-0.5*pi];
L(4) = Link('alpha', alpha(3),     'a', a(3),    'd', d(4),  'modified');
L(4).qlim=[-pi,pi];
L(5) = Link('alpha',alpha(4),      'a', a(4),    'd', d(5),  'modified');
L(5).qlim=[-pi,pi];
L(6) = Link('alpha', alpha(5),     'a', a(5),    'd', d(6),  'modified');
L(6).qlim = [-2*pi,2*pi];


robot0 = SerialLink(L,'name','engineer');

p=robot0.fkine(test);
x_in = p.t(1);
y_in = p.t(2);
z_in = p.t(3);
r11 = p.n(1);
r21 = p.n(2);
r31 = p.n(3);
r12 = p.o(1);
r22 = p.o(2);
r32 = p.o(3);
r13 = p.a(1);
r23 = p.a(2);
r33 = p.a(3);
R06=[r11 r12 r13;r21 r22 r23;r31 r32 r33];
% title('六轴机械臂模型');



%符号量
syms th1 th2 th3 th4 th5 th6 % x y z yaw pitch roll r
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

%% 变换矩阵
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
% T01_vpa=vpa(T01);
% T12_vpa=vpa(T12);
% T23_vpa=vpa(T23);
% T34_vpa=vpa(T34);
% T45_vpa=vpa(T45);
% T56_vpa=vpa(T56);

%% 逆解算

% x_in=0.295;
% y_in=0.09;
% z_in=0.345;
r_in=x_in^2+y_in^2+z_in^2;
% yaw=10;
% pitch=20;
% roll=30;
% yaw_in=roll;
% pitch_in=pi/2-pitch;
% roll_in=-yaw;
% yaw_in=45;
% pitch_in=45;
% roll_in=0;
% yaw_in_r=yaw_in/180*pi;
% pitch_in_r=pitch_in/180*pi;
% roll_in_r=roll_in/180*pi;


% R06 =[  cos(pitch_in_r)*cos(yaw_in_r),                                              -cos(pitch_in_r)*sin(yaw_in_r),                 sin(pitch_in_r);
%     cos(roll_in_r)*sin(yaw_in_r) + cos(yaw_in_r)*sin(pitch_in_r)*sin(roll_in_r), cos(roll_in_r)*cos(yaw_in_r) - sin(pitch_in_r)*sin(roll_in_r)*sin(yaw_in_r), -cos(pitch_in_r)*sin(roll_in_r);
%     sin(roll_in_r)*sin(yaw_in_r) - cos(roll_in_r)*cos(yaw_in_r)*sin(pitch_in_r), cos(yaw_in_r)*sin(roll_in_r) + cos(roll_in_r)*sin(pitch_in_r)*sin(yaw_in_r),  cos(pitch_in_r)*cos(roll_in_r)];

% R06=[1 0 0;0 cos(roll_in_r) -sin(roll_in_r);0 sin(roll_in_r) cos(roll_in_r)]*[cos(pitch_in_r) 0 sin(pitch_in_r);0 1 0;-sin(pitch_in_r) 0 cos(pitch_in_r)]*[cos(yaw_in_r) -sin(yaw_in_r) 0;sin(yaw_in_r) cos(yaw_in_r) 0;0 0 1]*[0 0 1;0 1 0;-1 0 0];
% R06=[1 0 0;0 cos(roll_in_r) -sin(roll_in_r);0 sin(roll_in_r) cos(roll_in_r)]*[cos(pitch_in_r) 0 sin(pitch_in_r);0 1 0;-sin(pitch_in_r) 0 cos(pitch_in_r)]*[cos(yaw_in_r) -sin(yaw_in_r) 0;sin(yaw_in_r) cos(yaw_in_r) 0;0 0 1];


% R06=[cos(yaw_in_r) -sin(yaw_in_r) 0;sin(yaw_in_r) cos(yaw_in_r) 0;0 0 1]*[cos(pitch_in_r) 0 sin(pitch_in_r);0 1 0;-sin(pitch_in_r) 0 cos(pitch_in_r)]*[1 0 0;0 cos(roll_in_r) -sin(roll_in_r);0 sin(roll_in_r) cos(roll_in_r)];
% R06_v=vpa(R06)
% pieper方法
% syms f1 f2 f3 g1 g2 g3
% p4=T34(:,4);
% f=T23*p4;
f=[d(4)*sa3*s3+a(2);
    -d(4)*sa3*ca2*c3;
    d(3)*ca2];

% solve theta3
% r=vpa(f(1)^2+f(2)^2+f(3)^2+a(1)^2+d(2)^2+2*d(2)*f(3))
% r_vpa=vpa(r)
% vpa(0.087025+0.345^2+0.0081-2*0.295*0.345*s3)
% theta3=asin((0.21415-r)/0.20355);

aa=0.21415-r_in;
b=-0.4071;
c=0.21415-r_in;
u1=(-b+sqrt(b^2-4*aa*c))/(2*aa);
u2=(-b-sqrt(b^2-4*aa*c))/(2*aa);
theta3_1=atan(u1)*2-pi
theta3_2=atan(u2)*2-pi
theta3=theta3_2;

% solve theta2
f_sol=[0.345-0.295*sin(theta3);
        0.295*cos(theta3);
        -0.09;
        1];
g=T12*f_sol;
k=[f_sol(1);
   -f_sol(2);
   f_sol(1)^2+f_sol(2)^2+f_sol(3)^2+a(1)^2+d(2)^2+2*d(2)*f_sol(3);
   f_sol(3)*ca1+d(2)*ca1];

% z=(k(1)*s2-k(2)*c2)*sa1+k(4);
% z=f2_sol*cos(th2) + f1_sol*sin(th2);
% z_vpa=vpa(z)
aa=f_sol(1);
b=f_sol(2);
u1=(aa+sqrt(aa^2+b^2-z_in^2))/(z_in+b);
u2=(aa-sqrt(aa^2+b^2-z_in^2))/(z_in+b);
theta2_1=atan(u1)*2;
theta2_2=atan(u2)*2;
if theta2_1>pi/2 || theta2_1<0
    theta2=theta2_2;
else
theta2=theta2_1;
end

%solve theta1
g_sol=[f_sol(1)*cos(theta2)-f_sol(2)*sin(theta2);
    0.09;
    f_sol(1)*sin(theta2)+f_sol(2)*cos(theta2);
    1];

aa=-g_sol(2);
b=g_sol(1);
u1=(aa+sqrt(aa^2+b^2-x_in^2))/(x_in+b);
u2=(aa-sqrt(aa^2+b^2-x_in^2))/(x_in+b);
theta1_1=atan(u1)*2;
theta1_2=atan(u2)*2;
if y_in>=0
    if (theta1_1>180) && (abs(theta1_1-180)>0.0001) || (theta1_1<0)&&(abs(theta1_1)>0.0001)
        theta1=theta1_2;
    else
        theta1=theta1_1;
    end
else
    if theta1_1>=180 || theta1_1<=0
        theta1=theta1_1;
    else
        theta1=theta1_2;
    end
end


% solve theta4 5 6
T01_sol=subs(T01,{th1},{theta1});
T12_sol=subs(T12,{th2},{theta2});
T23_sol=subs(T23,{th3},{theta3});

% T01_sol=[cos(theta1) -sin(theta1) 0 a0;
%     sin(theta1)*ca0 cos(theta1)*ca0 -sa0 -sa0*d(1);
%     sin(theta1)*sa0 cos(theta1)*sin(alpha0) ca0 ca0*d(1);
%     0 0 0 1];
% T12_sol=[cos(theta2) -sin(theta2) 0 a(1);
%     sin(theta2)*ca1 cos(theta2)*ca1 -sa1 -sa1*d(2);
%     sin(theta2)*sa1 cos(theta2)*sa1 ca1 ca1*d(2);
%     0 0 0 1];
% T23_sol=[cos(theta3) -sin(theta3) 0 a(2);
%     sin(theta3)*ca2 cos(theta3)*ca2 -sa2 -sa2*d(3);
%     sin(theta3)*sa2 cos(theta3)*sa2 ca2 ca2*d(3);
%     0 0 0 1];
R01_sol=vpa(T01_sol(1:3,1:3))
R12_sol=T12_sol(1:3,1:3)
R23_sol=T23_sol(1:3,1:3)
% 
% R03_sol=R01_sol*R12_sol*R23_sol;
% R36_sol=(R03_sol^-1)*R06;

% 求theta4=0时 R46

T340_sol=subs(T34,{th4},{0})
% T340_sol=[1 0 0 a(3);
%     0 ca3 -sa3 -sa3*d(4);
%     0 sa3 ca3 ca3*d(4);
%     0 0 0 1];
R340_sol=T340_sol(1:3,1:3);
R040_sol=R01_sol*R12_sol*R23_sol*R340_sol;
R36_sol=R040_sol'*R06;
% R040_sol_v=vpa(R040_sol)
% R03=R01_sol*R12_sol*R23_sol;
% R36_sol=(R03^-1)*R06;
R36_sol_v=vpa(R36_sol)

% zyz欧拉角

theta5_1=atan2(sqrt(R36_sol(3,1)^2+R36_sol(3,2)^2),R36_sol(3,3));
theta5_2=atan2(-sqrt(R36_sol(3,1)^2+R36_sol(3,2)^2),R36_sol(3,3));
% if theta5_2
% if theta5_1>0 && theta
theta5=theta5_1;
if abs(theta5)>0.00001
    theta4=atan2(R36_sol(2,3),R36_sol(1,3));
    theta6=atan2(R36_sol(3,2),-R36_sol(3,1));
    % theta4=atan(R36_sol(2,3)/R36_sol(1,3));
    % theta6=atan(R36_sol(3,2)/-R36_sol(3,1));
else
    theta4=0;
    theta6=0;
end

% 
% % theta5=acos(R36_sol(1,1)) 
% % theta4=atan2(R36_sol(1,2),R36_sol(1,3))
% % % theta6=atan2(R36_sol(2,3),R36_sol(2,2))
% % theta6=atan2(-R36_sol(3,1),R36_sol(2,1))


% theta1_d=vpa(theta1/pi*180);
% theta2_d=vpa(theta2/pi*180);
% theta3_d=vpa(theta3/pi*180);
% theta4_d=vpa(theta4/pi*180)
% theta5_d=vpa(theta5/pi*180)
% theta6_d=vpa(theta6/pi*180)

test
theta=double([theta1 theta2 theta3 theta4 theta5 theta6])
robot0.teach(theta);