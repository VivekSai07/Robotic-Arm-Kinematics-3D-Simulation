% Simulation of 2DOF robotic arm
clc
clear all
clearvars
clf
syms theta1 theta2
l1 = 1;
l2 = 1;
%specify the geometry of the robot
% DH parameters %%
% link-1
a1 = l1; alpha1 = 0; d1 = 0; theta1 = theta1;
% link-2
a2 = l2; alpha2 = 0; d2 = 0; theta2 = theta2;

H0_1 = DH(a1, alpha1, d1, theta1);
H1_2 = DH(a2, alpha2, d2, theta2);
H0_2 = H0_1 * H1_2;
Ax1 = H0_1(1,4);
Ay1 = H0_1(2,4);
Bx2 = simplify(H0_2(1,4));
By2 = simplify(H0_2(2,4));
t = 0;
dt = 0.1;
for i = 1:200
    t = t + dt;
    
    theta1 = pi/4 * sin(t);
    theta2 = pi/4 * cos(t);
    
    Ax1 = l1*cos(theta1);
    Ay1 = l1*sin(theta1);
    Az1 = 0;
    
    Bx2 = l2*cos(theta1 + theta2) + l1*cos(theta1);
    By2 = l2*sin(theta1 + theta2) + l1*sin(theta1);
    Bz2 = 0;
    
    v1 = [Ax1 Ay1 Az1]; v2 = [Bx2 By2 Bz2];
    txtend = ['x2 = ', num2str(Bx2),' , ','y2 = ', num2str(By2),' , ','z2 = ', num2str(Bz2)];
    %% plotting
    L1 = plot3([0 Ax1],[0 Ay1],[0 Az1],'color','b','LineWidth',2);
    hold on
    L2 = plot3([Ax1 Bx2],[Ay1 By2],[Az1 Bz2],'color','r','LineWidth',2);
    grid on
    %% setting square axis
    axlim = max([norm(v1) norm(v2)]);
    set(gca,'xlim',[-1 1]*axlim,'ylim',[-1 1]*axlim,'zlim',[-1 1]*axlim);
    grid on
    axis square
    %% plotting 0-lines
    hold on
    h1 = plot3(get(gca,'xlim'),[0 0],[0 0],'r--');
    h2 = plot3([0 0],get(gca,'ylim'),[0 0],'b--');
    h3 = plot3([0 0],[0 0],get(gca,'xlim'),'y--');
    set([h1,h2,h3],'color',[1 1 1]*.3);
    rotate3d on
    %
    %     Link1 = line([0,Ax1],[0,Ay1])
    %     Link2 = line([Ax1,Bx2],[Ay1,By2])
    %     axis([-3 3 -3 3]);
    %     title('~Forward Kinematics~');
    str1 = text(Bx2,By2,txtend,'FontSize',8,'HorizontalAlignment','left','VerticalAlignment','bottom');
    xlabel('~X-axis~');
    ylabel('~Y-axis~');
    zlabel('~Z-axis~');
    legend('Link-1','Link-2')
    grid on
    pause(0.01);
    delete(L1)
    delete(L2)
    delete(str1)
end
function [A] = DH(a, alpha, d, theta)
A = [cos(theta) -sin(theta)*round(cos(alpha)) sin(theta)*round(sin(alpha)) a*cos(theta);
    sin(theta) cos(theta)*round(cos(alpha)) -cos(theta)*round(sin(alpha)) a*sin(theta);
    0                 round(sin(alpha))         round(cos(alpha))          d       ;
    0                       0                         0                    1      ]
end