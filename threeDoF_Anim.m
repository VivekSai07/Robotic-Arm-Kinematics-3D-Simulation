% simulation of 3DOF robotic arm
clc
clear all
clearvars
clf
syms theta1 theta2 theta3 
l1 = 5;
l2 = 5;
l3 = 5;
%specify the geometry of the robot
% DH parameters %%
% link-1
a1 = 0; alpha1 = 90; d1 = l1; theta1 = theta1;
% link-2
a2 = 0; alpha2 = -90; d2 = l2; theta2 = theta2;
% link-3
a3 = 0; alpha3 = 0; d3 = l3; theta3 = theta3;

H0_1 = DH(a1, alpha1, d1, theta1);
H1_2 = DH(a2, alpha2, d2, theta2);
H2_3 = DH(a3, alpha3, d3, theta3);
H0_3 = H0_1 * H1_2 * H2_3;
x1 = H0_1(1,4);
y1 = H0_1(2,4);
z1 = H0_1(3,4);
x2 = H1_2(1,4); 
y2 = H1_2(2,4);
z2 = H1_2(3,4);
x3 = simplify(H0_3(1,4));
y3 = simplify(H0_3(2,4));
z3 = simplify(H0_3(3,4));
t = 0;
dt = 0.1;
for i = 1:500
    t = t + dt;
  
    theta1 = pi/4 * sin(t);
    theta2 = pi/2 * cos(t);
    theta3 = pi/4 * cos(t);
    x1 = 0;
    y1 = 0;
    z1 = l1;  
    
    x2 = l2*sin((pi*theta1)/180);
    y2 = -l2*cos((pi*theta1)/180);
    z2 = l1;
 
    x3 = l2*sin((pi*theta1)/180) - l3*cos((pi*theta1)/180)*sin((pi*theta2)/180);
    y3 = - l2*cos((pi*theta1)/180) - l3*sin((pi*theta1)/180)*sin((pi*theta2)/180);
    z3 = l1 + l3*cos((pi*theta2)/180);
    
    L1 = plot3([0,x1],[0,y1],[0,z1],'k','linew',3);
    hold on
    L2 = plot3([x1,x2],[y1,y2],[z1,z2],'r','linew',3);
    hold on
    L3 = plot3([x2,x3],[y2,y3],[z2,z3],'b','linew',3);
%     set([Link1 Link2 Link3],
%     plot3(Link1,Link2,Link3);
    %plot3(Link2,Link3);
%     axis([-10 10 -10 10]);
    txtend = ['x3 = ', num2str(x3),' , ','y3 = ', num2str(y3),' , ','z3 = ', num2str(z3)];
    str1 = text(x3,y3,z3,txtend,'FontSize',8,'HorizontalAlignment','left','VerticalAlignment','bottom');
    title('~Forward Kinematics~');
    xlabel('~X-axis~');
    ylabel('~Y-axis~');
    zlabel('~Z-axis~');
    xlim([-5, 5]);  % Set x-axis limits
    ylim([-10, 5]);  % Set y-axis limits
    zlim([-1, 10]);  % Set z-axis limits
    grid on
    pause(0.01);
    delete(L1)
    delete(L2)
    delete(L3)
    delete(str1)
end
function [A] = DH(a, alpha, d, theta)
 A = [cosd(theta) -sind(theta)*round(cosd(alpha)) sind(theta)*round(sind(alpha)) a*cosd(theta);
      sind(theta) cosd(theta)*round(cosd(alpha)) -cosd(theta)*round(sind(alpha)) a*sind(theta);
        0                 round(sind(alpha))         round(cosd(alpha))             d        ;
        0                       0                 0                     1       ];
end