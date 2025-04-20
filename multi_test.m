clear
clc

numSamples = 2000; % 采样点数
points = zeros(numSamples, 3); % 存储末端位置

% 参数
alpha0=0;
a0=0;
alpha=[ pi/2 0 -pi/2 pi/2 -pi/2];
a=[  0 0.345 0 0 0];
d=[0 0 0.09 0.295 0 0];

L(1) = Link('alpha', alpha0,         'a', a0,    'd', d(1),  'modified');
L(1).qlim=[-0.99*pi,0.99*pi];
L(2) = Link('alpha', alpha(1),      'a', a(1), 'd', d(2),  'modified');
L(2).qlim=[15/180*pi,140/180*pi];
L(3) = Link('alpha', alpha(2),         'a', a(2),'d', d(3),  'modified');
% L(3).qlim=[-250/180*pi,-45/180*pi];%-1.38  --  -0.25
L(3).qlim=[-250/180*pi,-0.51*pi];%-1.38  --  -0.25
L(4) = Link('alpha', alpha(3),     'a', a(3),    'd', d(4),  'modified');
L(4).qlim=[-pi,pi];
L(5) = Link('alpha',alpha(4),      'a', a(4),    'd', d(5),  'modified');
L(5).qlim=[-100/180*pi,80/180*pi];
L(6) = Link('alpha', alpha(5),     'a', a(5),    'd', d(6),  'modified');
L(6).qlim = [-2*pi,2*pi];
robot0 = SerialLink(L,'name','engineer');

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



% 各关节采样参数
joint_samples = 20; % 每个关节采样点数
q1_range = linspace(L(1).qlim(1), L(1).qlim(2), joint_samples);
q2_range = linspace(L(2).qlim(1), L(2).qlim(2), joint_samples);
q3_range = linspace(L(3).qlim(1), L(3).qlim(2), joint_samples);

% 初始化计数器
mismatch_count = 0;
total_configs = joint_samples^3;
processed_configs = 0;

% 遍历前三个关节组合
for q1 = q1_range
    for q2 = q2_range
        for q3 = q3_range
            % 当前关节配置
            current_q = [q1 q2 q3 0 0 0];

            % 计算正运动学（仅位置）
            T_desired = robot0.fkine(current_q);
            x_in = T_desired.t(1);
            y_in = T_desired.t(2);
            z_in = T_desired.t(3);

            % --- 您的逆运动学算法（前三个关节部分）开始 ---
            r_in = x_in^2 + y_in^2 + z_in^2;

            % % theta3
            f3=d3*ca2;
            % aa=(f3^2+a1^2+d2^2+2*d2*f3+d4^2+a2^2-r_in)
            % b=2*2*a2*d4*sa3
            % c=(f3^2+a1^2+d2^2+2*d2*f3+d4^2+a2^2-r_in)

            %
            % aa=0
            % b=2*a2*d4*sa3
            % c=f3^2+a1^2+d2^2+2*d2*f3+d4^2+a2^2-r_in
            %
            % tttttt=0;
            % aa=(r_in-d4^2-a2^2-2*a2*d4*sa3-f3^2-a1^2-d2^2-2*d2*f3)/(2*a2*d4*sa3);
            % tmp=1-aa^2
            % if abs(tmp)<0.000001
            %     tmp=0;
            % end
            % aa=(r_in-d4^2-a2^2-f3^2-a1^2-d2^2-2*d2*f3)



            % theta3_1=atan(u1)*2-pi;
            % theta3_2=atan(u2)*2-pi;
            % if theta3_1> -0.5*pi || theta3_1< -1.5*pi
            %     theta3=theta3_2;
            % else
            %     theta3=theta3_1;
            % end


            % theta3
            % theta3_1=asin((r_in-0.127125)/0.20355)
            % theta3_1=asin((0.21415-r_in)/0.20355)
            % theta3=theta3_1;

            %%%%%%%% theta3
            % % sin(theta)=a
            % aa=(0.21415-r_in)/0.20355
            % tmp=1-aa^2
            % theta3_1=atan2(sqrt(tmp),aa)
            % theta3_2=atan2(-sqrt(tmp),aa)

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




            % --- 您的逆运动学算法结束 ---

            % 计算逆解后的位姿
            % T_achieved = robot.fkine(theta);

            % 比较位置误差
            % position_error = norm(T_desired.t - T_achieved.t);


            % 判断是否匹配
            % if position_error > 1e-6
            %     mismatch_count = mismatch_count + 1;
            %     fprintf('Mismatch at q = [%.2f, %.2f, %.2f]\n', current_q);
            %     fprintf('  Original position: [%.4f, %.4f, %.4f]\n', T_desired.t);
            %     fprintf('  Solved position: [%.4f, %.4f, %.4f]\n', T_achieved.t);
            %     fprintf('  Position error: %.6f\n\n', position_error);
            % end
            theta=[theta1 theta2 theta3 0 0 0];
            error=current_q-theta;
            if ~((abs(error(1))<0.01) && (abs(error(2))<0.01 )&& (abs(error(3))<0.01))
                mismatch_count = mismatch_count + 1;
                points(mismatch_count,:) =[x_in,y_in,z_in]; % 提取x,y,z坐标
                fprintf('Mismatch at q = [%.2f, %.2f, %.2f,%.2f, %.2f, %.2f] Solve:[%.2f, %.2f, %.2f,%.2f, %.2f, %.2f]\n', current_q,theta);
            end
            % 进度显示
            processed_configs = processed_configs + 1;
            if mod(processed_configs, 100) == 0
                fprintf('Progress: %.1f%%\n', 100*processed_configs/total_configs);
            end
        end
    end
end

% 结果统计
fprintf('\nTotal configurations tested: %d\n', total_configs);
fprintf('Mismatched configurations: %d (%.2f%%)\n', mismatch_count, 100*mismatch_count/total_configs);

% 可视化最后一个不匹配案例（如果有）
if mismatch_count > 0
    figure;
    subplot(1,2,1);
    robot0.plot(current_q);
    title('Original Configuration');

    subplot(1,2,2);
    robot0.plot(theta);
    title('Solved Configuration');
end
figure;
scatter3(points(:,1), points(:,2), points(:,3), 5, 'filled', 'MarkerFaceAlpha', 0.3);
title('错误点');
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
grid on;
axis equal;
