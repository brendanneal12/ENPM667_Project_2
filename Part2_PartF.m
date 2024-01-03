%% Brendan Neal, Adam Lobo, and Hoang Pham
%% ENPM667 Project 2 (Final Project)
%% Part 2 Part F: Luenberger Observer
clear
clc

%% Setup
MM = 1000;
m1 = 100;
m2 = 100;
l1 = 20;
l2 = 10;
g = 9.81;

%A matrix
A_open_loop = [0, 1, 0, 0, 0, 0;
    0, 0, -m1*g/MM, 0, -m2*g/MM, 0;
    0, 0, 0, 1, 0, 0;
    0, 0, -(MM+m1)*g/(MM*l1), 0, -m2*g/(MM*l1), 0;
    0, 0, 0, 0, 0, 1;
    0, 0, -m1*g/(MM*l2), 0, -(MM+m2)*g/(MM*l2), 0];

%B matrix
B_open_loop = [0; 1/MM; 0; 1/(MM*l1); 0; 1/(MM*l2)];

%Observable Output Vectors
%Have to change the format to get graphs properly.
C1 = [1, 0, 0, 0, 0, 0];
C2 = [1,0,0,0,0,0; 0,0,0,0,1,0];
C3 = [1, 0, 0, 0, 0, 0 ; 0, 0, 1, 0, 0, 0 ;0, 0, 0, 0, 1, 0];

D = 0;

% LQR
% Selection of Q and R:
Q = [1000 0 0 0 0 0;
    0 1000 0 0 0 0;
    0 0 1000 0 0 0;
    0 0 0 1000 0 0
    0 0 0 0 1000 0
    0 0 0 0 0 1000];
R = 0.00001; % GOOD R

% Designing LQR Controller:
[K, P, EigenValues] = lqr(A_open_loop, B_open_loop, Q, R);

%% Placeing Poles and Getting L
%Placing Poles
des_poles = [-5, -4, -2, -8, -9, -3];
%Output Vector 1
L_C1=place(A_open_loop',C1',des_poles);
L_C1=L_C1';

%Output Vector 2
L_C2=place(A_open_loop',C2',des_poles);
L_C2=L_C2';

%Output Vector 3
L_C3=place(A_open_loop',C3',des_poles);
L_C3=L_C3';

%% Forming Closed Loop Systems

%First output vector
A_closed_loop_1=[(A_open_loop-B_open_loop*K) (B_open_loop*K); zeros(size(A_open_loop)) (A_open_loop-L_C1*C1)];
B_closed_loop_1=[ B_open_loop ;zeros(size(B_open_loop))];
C_closed_loop_1= [C1 zeros(size(C1))];
D_closed_loop_1=D;

%Second output vector
A_closed_loop_2=[(A_open_loop-B_open_loop*K) (B_open_loop*K); zeros(size(A_open_loop)) (A_open_loop-L_C2*C2)];
B_closed_loop_2=[ B_open_loop ;zeros(size(B_open_loop))];
C_closed_loop_2= [C2 zeros(size(C2))];
D_closed_loop_2=D;

%Third output vector
A_closed_loop_3=[(A_open_loop-B_open_loop*K) (B_open_loop*K); zeros(size(A_open_loop)) (A_open_loop-L_C3*C3)];
B_closed_loop_3=[ B_open_loop ;zeros(size(B_open_loop))];
C_closed_loop_3= [C3 zeros(size(C3))];
D_closed_loop_3=D;


%% Simulating First System
%Initial Conditions
X_0 = [3; 0; pi/4; 0; pi/4; 0; 0; 0; 0; 0; 0; 0];

Vector1_Sys=ss(A_closed_loop_1,B_closed_loop_1,C_closed_loop_1,D_closed_loop_1);
figure()
step(Vector1_Sys);
figure()
initial(Vector1_Sys,X_0);
grid on

%% Simulating Second System
Vector2_Sys=ss(A_closed_loop_2,B_closed_loop_2,C_closed_loop_2,D_closed_loop_2);
figure()
step(Vector2_Sys);
figure()
initial(Vector2_Sys,X_0);
grid on

%% Simulating Third System
Vector3_Sys=ss(A_closed_loop_3,B_closed_loop_3,C_closed_loop_3,D_closed_loop_3);
figure()
step(Vector3_Sys);
figure()
initial(Vector3_Sys,X_0);
grid on



