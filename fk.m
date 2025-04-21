clear
% 参数
alpha0=0;
a0=0;
alpha=[ pi/2 0 -pi/2 pi/2 -pi/2];
a=[  0 0.345 0 0 0];
d=[0 0 0.09 0.295 0 0];

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
% L(5).qlim=[-100/180*pi,80/180*pi];
L(5).qlim=[-0.5*pi,0.5*pi];
L(6) = Link('alpha', alpha(5),     'a', a(5),    'd', d(6),  'modified');
L(6).qlim = [-2*pi,2*pi];
robot0 = SerialLink(L,'name','engineer');

% 符号量
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
% ca0=cos(alpha0);
% sa0=sin(alpha0);
% ca1=cos(alpha(1));
% sa1=sin(alpha(1));
% ca2=cos(alpha(2));
% sa2=sin(alpha(2));
% ca3=cos(alpha(3));
% sa3=sin(alpha(3));
% ca4=cos(alpha(4));
% sa4=sin(alpha(4));
% ca5=cos(alpha(5));
% sa5=sin(alpha(5));
a1=a(1);
a2=a(2);
a3=a(3);
a4=a(4);
a5=a(5);
d1=d(1);
d2=d(2);
d3=d(3);
d4=d(4);
d5=d(5);
d6=d(6);
syms th1 th2 th3 th4 th5 th6

T01=[cos(th1) -sin(th1) 0 a0;
    sin(th1)*ca0 cos(th1)*ca0 -sa0 -sa0*d1;
    sin(th1)*sa0 cos(th1)*sa0 ca0 ca0*d1;
    0 0 0 1];

% T12
T12 = [cos(th2) -sin(th2) 0 a1;
    sin(th2)*ca1 cos(th2)*ca1 -sa1 -sa1*d2;
    sin(th2)*sa1 cos(th2)*sa1 ca1 ca1*d2;
    0 0 0 1];

% T23
T23 = [cos(th3) -sin(th3) 0 a2;
    sin(th3)*ca2 cos(th3)*ca2 -sa2 -sa2*d3;
    sin(th3)*sa2 cos(th3)*sa2 ca2 ca2*d3;
    0 0 0 1];

% T34
T34 = [cos(th4) -sin(th4) 0 a3;
    sin(th4)*ca3 cos(th4)*ca3 -sa3 -sa3*d4;
    sin(th4)*sa3 cos(th4)*sa3 ca3 ca3*d4;
    0 0 0 1];

% T45
T45 = [cos(th5) -sin(th5) 0 a4;
    sin(th5)*ca4 cos(th5)*ca4 -sa4 -sa4*d5;
    sin(th5)*sa4 cos(th5)*sa4 ca4 ca4*d5;
    0 0 0 1];

% T56
T56 = [cos(th6) -sin(th6) 0 a5;
    sin(th6)*ca5 cos(th6)*ca5 -sa5 -sa5*d6;
    sin(th6)*sa5 cos(th6)*sa5 ca5 ca5*d6;
    0 0 0 1];

T06=T01*T12*T23*T34*T45*T56