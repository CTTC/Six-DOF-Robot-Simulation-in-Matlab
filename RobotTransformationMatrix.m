
%% Initialization
clear all;clc;close all;
syms theta1 theta2 theta3 theta4 theta5 theta6
theta=[theta1 theta2 theta3 theta4 theta5 theta6];
d=[0 0 0.1 0.722 0 0.308];
a=[0 0.92 0.117 0 0 0];
alpha=[-pi/2 0 -pi/2 pi/2 -pi/2 0];

%% Transformation Matrix
for i=1:6
    A(:,:,i)=[cos(theta(i)) -sin(theta(i))*cos(alpha(i)) sin(theta(i))*sin(alpha(i)) a(i)*cos(theta(i));
        sin(theta(i)) cos(theta(i))*cos(alpha(i)) -cos(theta(i))*sin(alpha(i)) a(i)*sin(theta(i));
        0 sin(alpha(i)) cos(alpha(i)) d(i); 0 0 0 1];
end
displayA=vpa(A,3);
A1=A(:,:,1);
A2=A(:,:,2);
A3=A(:,:,3);
A4=A(:,:,4);
A5=A(:,:,5);
A6=A(:,:,6);
T6=A1*A2*A3*A4*A5*A6;  
T5=A1*A2*A3*A4*A5;     
T4=A1*A2*A3*A4;        
T3=A1*A2*A3;           
T2=A1*A2;              
T1=A1;                 

ZeroTSix=T6;
OneTSix=A2*A3*A4*A5*A6;
TwoTSix=A3*A4*A5*A6;
ThreeTSix=A4*A5*A6;
FourTSix=A5*A6;
FiveTSix=A6;

T(:,:,1)=ZeroTSix;
T(:,:,2)=OneTSix;
T(:,:,3)=TwoTSix;
T(:,:,4)=ThreeTSix;
T(:,:,5)=FourTSix;
T(:,:,6)=FiveTSix;

%% Jaccobian Matrix
%Jaccobian Matrix which is relative to the six joint coordinate system
for j=1:6
    B=T(:,:,j);
    nx=B(1,1);
    ny=B(2,1);
    nz=B(3,1);
    ox=B(1,2);
    oy=B(2,2);
    oz=B(3,2);
    ax=B(1,3);
    ay=B(2,3);
    az=B(3,3);
    px=B(1,4);
    py=B(2,4);
    pz=B(3,4);
    J(1,j)=ny*px-nx*py;
    J(2,j)=oy*px-ox*py;
    J(3,j)=ay*px-ax*py;
    J(4,j)=nz;
    J(5,j)=oz;
    J(6,j)=az;
end
displayJ=vpa(J,3);

%% Workspace   
workspace_x=0.92*cos(theta1)*cos(theta2)-0.1*sin(theta1)-...
    0.308*cos(theta5)*(cos(theta1)*cos(theta2)*sin(theta3)+...
    cos(theta1)*cos(theta3)*sin(theta2))-...
    0.308*sin(theta5)*(sin(theta1)*sin(theta4)-...
    1.0*cos(theta4)*(cos(theta1)*sin(theta2)*sin(theta3)-...
    1.0*cos(theta1)*cos(theta2)*cos(theta3)))-...
    0.117*cos(theta1)*sin(theta2)*sin(theta3)+...
    0.117*cos(theta1)*cos(theta2)*cos(theta3)-...
    0.722*cos(theta1)*cos(theta2)*sin(theta3) -...
    0.722*cos(theta1)*cos(theta3)*sin(theta2);  %workspace_x=vpa(T6(1,4),3)

workspace_y=0.1*cos(theta1) + 0.92*cos(theta2)*sin(theta1) +...
    0.308*sin(theta5)*(cos(theta1)*sin(theta4) + ...
    cos(theta4)*(sin(theta1)*sin(theta2)*sin(theta3) - ...
    1.0*cos(theta2)*cos(theta3)*sin(theta1))) - ...
    0.308*cos(theta5)*(cos(theta2)*sin(theta1)*sin(theta3) +...
    cos(theta3)*sin(theta1)*sin(theta2)) -...
    0.722*cos(theta2)*sin(theta1)*sin(theta3) -...
    0.722*cos(theta3)*sin(theta1)*sin(theta2) - ...
    0.117*sin(theta1)*sin(theta2)*sin(theta3) +...
    0.117*cos(theta2)*cos(theta3)*sin(theta1);  %workspace_y=vpa(T6(2,4),3) 

workspace_z=0.722*sin(theta2)*sin(theta3) - 0.722*cos(theta2)*cos(theta3)-...
    0.117*cos(theta2)*sin(theta3) - 0.117*cos(theta3)*sin(theta2) -...
    0.92*sin(theta2) - 0.308*cos(theta5)*(cos(theta2)*cos(theta3) -...
    1.0*sin(theta2)*sin(theta3)) + ...
    0.308*cos(theta4)*sin(theta5)*(cos(theta2)*sin(theta3) +...
    cos(theta3)*sin(theta2)); %workspace_z=vpa(T6(3,4),3)

