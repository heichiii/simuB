clear
% 参数
% 设置显示格式为短格式
format short
alpha0 = 0;
a0 = 0;
alpha = [pi/2  0  pi/2  -pi/2  pi/2];
a     = [0  0.345  0  0  0];
d     = [0  0  -0.09305  0.295  0  0.122];
test        = [-pi/3  2*pi/6  pi*0.95  pi/6  pi/3  4*pi/6];
test_link4  = [test(1)  test(2)  test(3)  test(4)];
L(1) = Link('alpha',  alpha0,  'a',  a0,  'd',  d(1),  'offset',  -pi/2,  'modified');
L(1).qlim=[-pi,pi];
L(2) = Link('alpha',  alpha(1),  'a',  a(1),  'd',  d(2),  'offset',  0,  'modified');
L(2).qlim=[0,2.792];
L(3) = Link('alpha',  alpha(2),  'a',  a(2),  'd',  d(3),  'offset',  pi/2,  'modified');
L(3).qlim=[0,pi];
L(4) = Link('alpha',  alpha(3),  'a',  a(3),  'd',  d(4),  'offset',  0,  'modified');
L(4).qlim=[-pi,pi];
L(5) = Link('alpha',  alpha(4),  'a',  a(4),  'd',  d(5),  'offset',  0,  'modified');
L(5).qlim=[0,pi];
L(6) = Link('alpha',  alpha(5),  'a',  a(5),  'd',  d(6),  'offset',  0,  'modified');
L(6).qlim = [-pi,pi];
robot = SerialLink(L,'name','engineer');
robot.teach(test);
robot.display()

%由于正解算只返回R06,故补充R04
L_link4(1) = Link('alpha',  alpha0,  'a',  a0,  'd',  d(1),  'offset',  -pi/2,  'modified');
L_link4(1).qlim=[-pi,pi];
L_link4(2) = Link('alpha',  alpha(1),  'a',  a(1),  'd',  d(2),  'offset',  0,  'modified');
L_link4(2).qlim=[0,pi];
L_link4(3) = Link('alpha',  alpha(2),  'a',  a(2),  'd',  d(3),  'offset',  pi/2,  'modified');
L_link4(3).qlim=[0,pi];
L_link4(4) = Link('alpha',  alpha(3),  'a',  a(3),  'd',  d(4),  'offset',  0,  'modified');
L_link4(4).qlim=[-pi,pi];
robot_link4 = SerialLink(L_link4,'name','engineer_link4');

% 符号量
sa0 = sin(alpha0);
sa1 = sin(alpha(1));
sa2 = sin(alpha(2));
sa3 = sin(alpha(3));
sa4 = sin(alpha(4));
sa5 = sin(alpha(5));
ca0 = cos(alpha0);
ca1 = cos(alpha(1));
ca2 = cos(alpha(2));
ca3 = cos(alpha(3));
ca4 = cos(alpha(4));
ca5 = cos(alpha(5));
a1 = a(1);
a2 = a(2);
a3 = a(3);
a4 = a(4);
a5 = a(5);
d1 = d(1);
d2 = d(2);
d3 = d(3);
d4 = d(4);
d5 = d(5);
d6 = d(6);

% 正解算
p = robot.fkine(test);
x_in = p.t(1)
y_in = p.t(2)
z_in = p.t(3)
r_in = x_in^2 + y_in^2 + z_in^2;
r11 = p.n(1);
r21 = p.n(2);
r31 = p.n(3);
r12 = p.o(1);
r22 = p.o(2);
r32 = p.o(3);
r13 = p.a(1);
r23 = p.a(2);
r33 = p.a(3);
R06 = [r11 r12 r13;r21 r22 r23;r31 r32 r33]

p_link4 = robot_link4.fkine(test_link4);
x_in_link4 = p_link4.t(1);
y_in_link4 = p_link4.t(2);
z_in_link4 = p_link4.t(3);
r_in_link4 = x_in_link4^2 + y_in_link4^2 + z_in_link4^2;
r11_link4 = p_link4.n(1);
r21_link4 = p_link4.n(2);
r31_link4 = p_link4.n(3);
r12_link4 = p_link4.o(1);
r22_link4 = p_link4.o(2);
r32_link4 = p_link4.o(3);
r13_link4 = p_link4.a(1);
r23_link4 = p_link4.a(2);
r33_link4 = p_link4.a(3);
R04_link4 = [r11_link4 r12_link4 r13_link4;r21_link4 r22_link4 r23_link4;r31_link4 r32_link4 r33_link4];

%逆解算
% theta3
syms theta3;
s3 = sin(theta3);
c3 = cos(theta3);

% f1 = a3*c3 + d4*sa3*s3 + a2;
% f2 = a3*ca2*s3 - d4*sa3*ca2*c3 - d4*sa2*ca3 - d3*sa2;
% f3 = a3*sa2*s3 - d4*sa3*sa2*c3 + d4*ca2*ca3 + d3*ca2;
f1 = d4*s3 + a2;
f2 = - d4*c3;
f3 = d3;

%r = f1*f1 + f2*f2 + f3*f3 + a1*a1 + d2*d2 + 2*d2*f3 + 2*a1*(c2*f1 - s2*f2);

k1 = f1;
k2 = -f2;
% k3 = f1*f1 + f2*f2 + f3*f3 + a1*a1 + d2*d2 + 2*d2*f3;
k3 = a1*a1 + a2*a2 + (d2 + d3)*(d2 + d3) + d4*d4 + 2*a2*d4*s3;
k4 = f3*ca1 + d2*ca1;

r = k3; % a1=0
a = (r_in_link4 - (a2*a2 + d3*d3 + d4*d4)) / (2*a2*d4);

theta3_1 = atan2(sqrt(1 - a*a),a);
theta3_2 = -atan2(sqrt(1 - a*a),a);

theta3= theta3_1;

% theta2
syms theta2;
s2 = sin(theta2);
c2 = cos(theta2);
s3 = sin(theta3);
c3 = cos(theta3);

%解析法,和几何法一样
% f1 = d4*s3 + a2;
% f2 = - d4*c3;
% k1 = f1;
% k2 = -f2;
% k4 = 0;
% z = (k1*s2 - k2*c2)*sa1 + k4;

%几何法:b*s2 + a*c2 = z
a = d4*s3;
b = a2 + d4*c3;
c = z_in_link4;
d = sqrt(a*a + b*b - c*c);
theta2_1 = atan2(b, a) + atan2(d, c)
theta2_2 = atan2(b, a) - atan2(d, c)


if theta2_2 > pi/2 || theta2_2 < 0.06
    theta2 = theta2_1
else
    theta2 = theta2_2
end

% theta1
s2 = sin(theta2);
c2 = cos(theta2);

%解析法,不好用
% g1 = c2*f1 - s2*f2 + a1;
% g2 = s2*ca1*f1 + c2*ca1*f2 - sa1*f3 - d2*sa1;
% g3 = s2*sa1*f1 + c2*sa1*f2 + ca1*f3 + d2*ca1;
% g1 = c2*f1 - s2*f2;
% g2 = -f3 - d2;
% g3 = s2*f1 + c2*f2;

%几何法,加判断
l1 = - a2*c2;
l2 = - d4*cos(theta2 + theta3);
l = l1 + l2;
x = x_in_link4;
y = y_in_link4;
% if l < 0
%     x = -x;
%     y = -y;
% end

% - s1*l - d3*c1
% c1*l - d3*s1
a = (d3*y + x*l) / (-l*l - d3*d3);
b = (y*l - d3*x) / (l*l + d3*d3);

theta1 = atan2(a, b);

% theta 4 5 6
s1 = sin(theta1);
c1 = cos(theta1);
c4 = 1;
s4 = 0;

R04 = [-s1*(c2*s3 + c3*s2),  - c1*c4 + s4*(c2*c3*s1 - s1*s2*s3),  c1*s4 + c4*(c2*c3*s1 - s1*s2*s3);
       c1*(c2*s3 + c3*s2),   - c4*s1 - s4*(c1*c2*c3 - c1*s2*s3),  -c4*(c1*c2*c3 - c1*s2*s3) + s1*s4;
       c2*c3 - s2*s3     ,   -s4*(c2*s3 + c3*s2),                 c4*(c2*s3 + c3*s2)];

R406 = R04'*R06;

theta5=atan2(sqrt(R406(1,3)^2+R406(2,3)^2), R406(3,3));

s5 = sin(theta5);
c5 = cos(theta5);
if abs(theta5)>0.00001
    theta4 = atan2(R406(2,3)/s5, R406(1,3)/s5);
    theta6 = atan2(R406(3,2)/s5, -R406(3,1)/s5);
else
    theta4=0;
    theta6=0;
end
s4 = sin(theta4);
c4 = cos(theta4);
s6 = sin(theta6);
c6 = cos(theta6);
R04 = [-s1*(c2*s3 + c3*s2),  - c1*c4 + s4*(c2*c3*s1 - s1*s2*s3),  c1*s4 + c4*(c2*c3*s1 - s1*s2*s3);
       c1*(c2*s3 + c3*s2),   - c4*s1 - s4*(c1*c2*c3 - c1*s2*s3),  -c4*(c1*c2*c3 - c1*s2*s3) + s1*s4;
       c2*c3 - s2*s3     ,   -s4*(c2*s3 + c3*s2),                 c4*(c2*s3 + c3*s2)];

test
% theta = robot.ikine(p) %逆解算验证
theta = [theta1 theta2 theta3 theta4 theta5 theta6]
% robot.teach(theta);