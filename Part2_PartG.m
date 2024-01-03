%% Brendan Neal, Adam Lobo, and Hoang Pham
%% ENPM667 Project 2 (Final Project)
%% Part 2 Part G: LQG Controller
clear
clc

%% Design
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

% Smallest Output Vector
C_open_loop = [1,0,0,0,0,0];

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

% Setting up LQR System
LQR_System = ss(A_open_loop,[B_open_loop B_open_loop],C_open_loop,[zeros(1,1) zeros(1,1)]);

% Define covariance of disturbance (ensure gaussian) 
disturbance = 0.01;
% Define covariance of noise (ensure gaussian)
noise = 1;

% Obtaining L from Kalman Filter
[~,L,~] = kalman(LQR_System, disturbance, noise, [], 1, 1);

% Forming Closed Loop System
A_closed_loop = [A_open_loop-B_open_loop*K B_open_loop*K;zeros(size(A_open_loop)) A_open_loop-L*C_open_loop];
B_closed_loop = zeros(12,1);
C_closed_loop = [C_open_loop zeros(size(C_open_loop))];
D = 0;

%Use state space function to set up LQG System
LQG_System = ss(A_closed_loop, B_closed_loop, C_closed_loop, D);

% Setting Simulation Conditions
timevector = 0:0.1:200;
X_0 = [3; 0; pi/4; 0; pi/4; 0; 0; 0; 0; 0; 0; 0];
F = zeros(size(timevector));

% Simulating
[Y,~,X] = lsim(LQG_System,F,timevector, X_0);

%% Plotting
% State Variables
figure()
subplot(6,1,1);
plot(timevector,X(:,1),'k')
ylabel('X (m)')
xlabel('Time (s)')
grid on;

subplot(6,1,2);
plot(timevector,X(:,2),'k')
grid on;
ylabel('X Dot (m/s)')
xlabel('Time (s)')

subplot(6,1,3);
plot(timevector,X(:,3),'r')
grid on;
ylabel('Theta 1 (rad)')
xlabel('Time (s)')

subplot(6,1,4);
plot(timevector,X(:,4),'r')
grid on;
ylabel('Theta 1 Dot (rad/s)')
xlabel('Time (s)')

subplot(6,1,5);
plot(timevector,X(:,5),'b')
grid on;
ylabel('Theta 2 (rad)')
xlabel('Time (s)')

subplot(6,1,6);
plot(timevector,X(:,6),'b')
grid on;
ylabel('Theta 2 Dot (rad/s)')
xlabel('Time (s)')

% Output
figure()
plot(timevector,Y(:,1),'k')
ylabel('X (m)')
xlabel('Time (s)')
title('X(t) vs. Time Under LQG Control')
grid on;


