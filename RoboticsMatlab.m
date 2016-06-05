%% General Information
% Function: Perform forward and inverse kinematics for a six-DOF robot
% Author: Tao Chen
% E-mail:chentao904@163.com
% Department:School of Mechanical Engineering, 
%            Shanghai Jiao Tong University
% Date:June 28th,2015
%% Initialization
clear all;clc;close all;
L(1)=Link([0,0,0,-pi/2,0]);
L(2)=Link([0,0,0.92,0,0]);
L(3)=Link([0,0.1,-0.117,pi/2,0]);
L(4)=Link([0,0.722,0,pi/2,0]);
L(5)=Link([0,0,0,-pi/2,0]);
L(6)=Link([0,0.308,0,0,0]);
six_link=SerialLink(L,'name','OurRobot');
six_link.base=transl(0,0,0.76);
qz=[0 0 0 0 0 0];
qr=[0 -pi/2 pi/2 0 0 0];
qs=[0 0 pi/2 0 0 0];
qn=[0 -pi/4 pi 0 -pi/4 0];

%% Forward Kinematics
Fstretch=six_link.fkine(qs);
Fzero=six_link.fkine(qz);
Fready=six_link.fkine(qr);
Fnominal=six_link.fkine(qn);
%% Figure 1_zero
figure(1);
six_link.plot(qz);  % zero angle
title('zero angle');
%% Figure 2_ready
figure(2);
six_link.plot(qr); %ready, the arm is straight and vertical
title('ready, the arm is straight and vertical');
%% Figure 3_stretch
figure(3);
six_link.plot(qs);  % stretch,the arm is straight and horizontal
title('stretch,the arm is straight and horizontal');
%% Figure 4_nominal
figure(4); 
six_link.plot(qn); % nominal, the arm is in dextrous working pose
title('nominal, the arm is in dextrous working pose');

%% Inverse kinematics_nru
 figure(5);
 A=[-1 0 0 1.2438;0 1 0 0.1;0 0 -1 0.6747;0 0 0 1];
 q1=six_link.ikine6s(A,'nru');
 six_link.plot(q1);
 title('ikine6s,nru');
%% Inverse kinematics_nrd
figure(6);
q1=six_link.ikine6s(A,'nrd');
six_link.plot(q1);
title('ikine6s,nrd');
%% Inverse kinematics_nlu
figure(7);
q1=six_link.ikine6s(A,'nlu');
six_link.plot(q1);
title('ikine6s,nlu');
%% Inverse kinematics_nld
figure(8);
q1=six_link.ikine6s(A,'nld');
six_link.plot(q1);
title('ikine6s,nld');
%% Inverse kinematics_fru
figure(9);
q1=six_link.ikine6s(A,'fru');
six_link.plot(q1);
title('ikine6s,fru');
%% Inverse kinematics_frd
figure(10);
q1=six_link.ikine6s(A,'frd');
six_link.plot(q1);
title('ikine6s,frd');
%% Inverse kinematics_flu
figure(11);
q1=six_link.ikine6s(A,'flu');
six_link.plot(q1);
title('ikine6s,flu');
%% Inverse kinematics_fld
figure(12);
q1=six_link.ikine6s(A,'fld');
six_link.plot(q1);
title('ikine6s,fld');
%% Jaccobian Matrix
J=six_link.jacob0(qn);  %Jaccobian matrix at qn pose
%% Circular Trajectory
t=[0:0.05:2]';
q1=[0.1412   -0.1665   -4.4891    3.1416    1.5140    0.1412];
q2=[6.0022    0.1801   -4.4905    3.1416    1.1689   -0.2810];
figure(13);
qtraj=jtraj(q1,q2,t);
for j=20:size(qtraj,1)
    T=six_link.fkine(qtraj(j,:));
    jta(j,:)=[T(1,4),T(2,4),T(3,4)];
    six_link.plot(qtraj(j,:));
    hold on;
    plot2(jta(j,:),'r.');
end
title('Circular Trajectory','Fontsize',20);
%% Linear Trajectory
t=[0:0.1:2]';
T1=transl(0.4,0.3,1.0)*trotx(pi);
T2=transl(1.4,-0.3,0.5)*trotx(pi);
Ts=ctraj(T1,T2,length(t));
qc=six_link.ikine(Ts);
tt=zeros(size(Ts,3),3);
figure(14);
for j=1:size(Ts,3)
    tt(j,:)=[Ts(1,4,j) Ts(2,4,j) Ts(3,4,j)];
    six_link.plot(qc(j,:));
    hold on;
    plot2(tt(j,:),'r.');
end
title('ֱ�߹켣','Fontsize',20);

%% Spiral Line Trajectory
t=0:0.5:6*pi;
x=1+0.5*sin(t);
x=x';
y=-1+0.5*cos(t);
y=y;
z=0.4+t/10;
z=z';
m=length(t);
for i=1:m
    T(:,:,i)=transl(x(i),y(i),z(i))*trotx(pi);
end
figure(15);
plot3(x,y,z,'r.');
hold on;
plot3(x,y,z,'b');
hold on;
q=six_link.ikine6s(T);
six_link.plot(q);
title('Spline Trajectory','Fontsize',20);