clear
% 参数
alpha0=0;
a0=0;
alpha=[ pi/2 0 -pi/2 pi/2 -pi/2];
a=[  0 0.345 0 0 0];
d=[0 0 0.09 0.295 0 0];
% test=[-0.25*pi 0.49*pi -1.1*pi 0.9*pi 0.2*pi 0];
test=[0.16, 2, -pi 0.25*pi 0.25*pi 0];
% test=[0 0.5*pi -pi,0,0,0];
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

robot1 = SerialLink(L,'name','engineer2');
% robot0.teach(test);
% 符号量
ca0=cos(alpha0);
sa0=sin(alpha0);
ca1=cos(alpha(1));
sa1=sin(alpha(1));
ca2=cos(alpha(2));
sa2=sin(alpha(2));
ca3=cos(alpha(3));
sa3=sin(alpha(3));
ca4=cos(alpha(4));
sa4=sin(alpha(4));
ca5=cos(alpha(5));
sa5=sin(alpha(5));
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
% 正解
p=robot0.fkine(test);
x_in = p.t(1);
y_in = p.t(2);
z_in = p.t(3);




r_in=x_in^2+y_in^2+z_in^2;
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
yaw_in=atan2(-r12,r11);
pitch_in=atan2(r13,r11/cos(yaw_in));
roll_in=atan2(-r23,r33);


% % theta3
f3=d3*ca2;
% a*cos(theta)+b*sin(theta)=c
aa=0;
b=0.20355;
c=0.21415-r_in;
theta3_1=atan2(b,aa)+atan2(sqrt(aa^2+b^2-c^2),c);
theta3_2=atan2(b,aa)-atan2(sqrt(aa^2+b^2-c^2),c);
% if theta3_1> -0.5*pi || theta3_1< -1.5*pi
%     theta3=theta3_2;
% else
%     theta3=theta3_1;
% end
theta3=theta3_1-2*pi;



% theta2
s3=sin(theta3);
c3=cos(theta3);
% b=d4*sa3*ca2*c3
% a=d4*sa3*s3+a2
% c=d3*ca2*ca1
% d=d2*ca1
aa=d4*sa3*ca2*c3+d3*ca2*ca1+d2*ca1-z_in;
b=2*(d4*sa3*s3+a2);
c=d3*ca2*ca1+d2*ca1-(d4*sa3*ca2*c3)-z_in;
u1=(-b+sqrt(b^2-4*aa*c))/(2*aa);
u2=(-b-sqrt(b^2-4*aa*c))/(2*aa);
theta2_1=atan(u1)*2;
theta2_2=atan(u2)*2;
flag=1;
if theta2_1<L(2).qlim(1)-0.1 || theta2_1>L(2).qlim(2)+0.1
    theta2=theta2_2;flag=0;

elseif theta2_2<L(2).qlim(1)-0.1 || theta2_2>L(2).qlim(2)+0.1 
        theta2=theta2_1;flag=0;
else
    theta2=theta2_1;
end

% if theta2_1>pi/2 || theta2_1<0
%     theta2=theta2_2;
% else
% theta2=theta2_1;
% end


% theta1
s2=sin(theta2);
c2=cos(theta2);
f1=d4*sa3*s3+a2;
f2=-d4*sa3*ca2*c3;

% x=c1*g1-s1*g2
g1=c2*f1-s2*f2+a1;
g2=s2*ca1*f1+c2*ca1*f2-sa1*f3-d2*sa1;
% aa=x_in+g1;
% b=2*g2;
% c=x_in-g1;
% u1=(-b+sqrt(b^2-4*aa*c))/(2*aa);
% u2=(-b-sqrt(b^2-4*aa*c))/(2*aa);
% theta1_1=atan(u1)*2;
% theta1_1_d=theta1_1/pi*180
% theta1_2=atan(u2)*2;
% theta1_2_d=theta1_2/pi*180
% theta1=theta1_2;
% y=s1*g1+c1*g2

aa=g1;
b=g2;
c=x_in;
d=y_in;
theta1_1=atan2(aa*d-b*c,aa*c+b*d);
% if (x_in<=0 && abs(theta1_1)>0.5*pi)
%     theta1=theta1_1;
% 
theta1=theta1_1;
if flag
if (y_in>=0)
    if((((x_in>=0.09) && (abs(theta1_1)<=0.5*pi)) || (x_in<0.09 && (abs(theta1_1)>0.5*pi) && (abs(theta1_1)<=pi))))
        theta1=theta1_1;
    else
        theta2=theta2_2;
        s2=sin(theta2);
        c2=cos(theta2);
        f1=d4*sa3*s3+a2;
        f2=-d4*sa3*ca2*c3;
        g1=c2*f1-s2*f2+a1;
        g2=s2*ca1*f1+c2*ca1*f2-sa1*f3-d2*sa1;
        aa=g1;
        b=g2;
        c=x_in;
        d=y_in;
        theta1_2=atan2(aa*d-b*c,aa*c+b*d);
        theta1=theta1_2;
    end
else
    if((x_in>=-0.09) && (theta1_1>=-0.51*pi)) || (x_in<-0.09 && (theta1_1<-0.5*pi) && (theta1_1>=-pi))
        theta1=theta1_1;
    else
        theta2=theta2_2;
        s2=sin(theta2);
        c2=cos(theta2);
        f1=d4*sa3*s3+a2;
        f2=-d4*sa3*ca2*c3;
        g1=c2*f1-s2*f2+a1;
        g2=s2*ca1*f1+c2*ca1*f2-sa1*f3-d2*sa1;
        aa=g1;
        b=g2;
        c=x_in;
        d=y_in;
        theta1_2=atan2(aa*d-b*c,aa*c+b*d);
        theta1=theta1_2;
    end
end

end

% if (((x_in>=0) && (abs(theta1_1)<=0.5*pi)) || (x_in<0 && (abs(theta1_1)>0.5*pi) && (abs(theta1_1)<pi)))
%     theta1=theta1_1;
% else
%     theta2=theta2_2;
%     s2=sin(theta2);
%     c2=cos(theta2);
%     f1=d4*sa3*s3+a2;
%     f2=-d4*sa3*ca2*c3;
%     g1=c2*f1-s2*f2+a1;
%     g2=s2*ca1*f1+c2*ca1*f2-sa1*f3-d2*sa1;
%     aa=g1;
%     b=g2;
%     c=x_in;
%     d=y_in;
%     theta1_2=atan2(aa*d-b*c,aa*c+b*d);
%     theta1=theta1_2;
% end




% if y_in>=0
%     if (theta1_1>180) && (abs(theta1_1-180)>0.0001) || (theta1_1<0)&&(abs(theta1_1)>0.0001)
%         theta1=theta1_2;
%     else
%         theta1=theta1_1;
%     end
% else
%     if theta1_1>=180 || theta1_1<=0
%         theta1=theta1_1;
%     else
%         theta1=theta1_2;
%     end
% end

% theta 4 5 6
s1=sin(theta1);
c1=cos(theta1);
c4=1;
s4=0;
R04 =[c4*(c1*c2*c3 - c1*s2*s3) - s1*s4, - c4*s1 - s4*(c1*c2*c3 - c1*s2*s3), -c1*(c2*s3 + c3*s2);
c1*s4 + c4*(c2*c3*s1 - s1*s2*s3),   c1*c4 - s4*(c2*c3*s1 - s1*s2*s3), -s1*(c2*s3 + c3*s2);
              c4*(c2*s3 + c3*s2),                -s4*(c2*s3 + c3*s2),       c2*c3 - s2*s3];

R406=R04'*R06;
theta5_1=atan2(sqrt(R406(1,3)^2+R406(2,3)^2),R406(3,3));
theta5_2=atan2(-sqrt(R406(1,3)^2+R406(2,3)^2),R406(3,3));
%theta5=atan(sqrt(R406(1,3)^2+R406(2,3)^2)/R406(3,3));
% theta5=theta5_2;

% if pitch_in>0 && theta5_1<pi/3


theta5=theta5_1;
if abs(theta5)>0.00001
    theta4=atan2(-(R406(2,3)/sin(theta5)),-(R406(1,3)/sin(theta5)));
    theta6=atan2(-(R406(3,2)/sin(theta5)),-(-R406(3,1)/sin(theta5)));
else
    theta4=0;
    theta6=0;
end

if(abs(theta4)>pi/2)
    theta5=theta5_2;
     theta4=atan2(-(R406(2,3)/sin(theta5)),-(R406(1,3)/sin(theta5)));
    theta6=atan2(-(R406(3,2)/sin(theta5)),-(-R406(3,1)/sin(theta5)));
end
% if pitch_in>0 && theta5_1<(80/180*pi)
%     theta5=theta5_1;
% else
%     theta5=theta5_2;
% end
% 
% if abs(theta5)>0.00001
%     theta4=atan2(-(R406(2,3)/sin(theta5)),-(R406(1,3)/sin(theta5)));
%     theta6=atan2(-(R406(3,2)/sin(theta5)),-(-R406(3,1)/sin(theta5)));
% else
%     theta4=0;
%     theta6=0;
% end

test;
test_deg=test/pi*180
theta=[theta1 theta2 theta3 theta4 theta5 theta6];
theta_deg=theta/pi*180
% robot0.teach();
% title('theta');
figure;
robot1.teach(test);
title('test');
