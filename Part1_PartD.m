%% Brendan Neal, Adam Lobo, and Hoang Pham
%% ENPM667 Project 2 (Final Project)
%% Part 1 Part D: LQR Controller
clear
clc

%% Controllability Check

% Parameters
MM = 1000;
m1 = 100;
m2 = 100;
l1 = 20;
l2 = 10;
g = 9.81;

%A matrix
A = [0, 1, 0, 0, 0, 0;
    0, 0, -m1*g/MM, 0, -m2*g/MM, 0;
    0, 0, 0, 1, 0, 0;
    0, 0, -(MM+m1)*g/(MM*l1), 0, -m2*g/(MM*l1), 0;
    0, 0, 0, 0, 0, 1;
    0, 0, -m1*g/(MM*l2), 0, -(MM+m2)*g/(MM*l2), 0];

%B matrix
B = [0; 1/MM; 0; 1/(MM*l1); 0; 1/(MM*l2)];

%Controllability Matrix
CNTR = [B, A*B, A^2*B, A^3*B, A^4*B, A^5*B];

%Output controllability matrix (simplified)
disp('The controllability matrix is:')
CNTR

%Rank of controllability matrix: < 6 = not controllable, = 6 = controllable
disp('The rank of the controllability matrix is:')
rank(CNTR)

%% Design of the LQR Controller -- Linearized System
%% First-Pass Design
%Initial Conditions
X_0 = [0, 0, pi/4, 0, pi/4, 0];
C = eye(6);
D = 0;


% Initial Selection of Q and R:
Q = [1 0 0 0 0 0;
    0 1 0 0 0 0;
    0 0 1 0 0 0;
    0 0 0 1 0 0
    0 0 0 0 1 0
    0 0 0 0 0 1];
R = 1;

% Designing LQR Controller:
[K, ~, EigenValues] = lqr(A, B, Q, R);

% Simulating LQR Test #1 using built in 
LQR_Test_1 = ss(A-(B*K),B, C, D);

% Plotting Response
 figure()
 initial(LQR_Test_1, X_0)

 disp('Eigenvalues for Lyapunovs Indirect Method:')
 disp(EigenValues)
%% Area for Adjusting Q and R
%Initial Conditions
X_0 = [0, 0, pi/4, 0, pi/4, 0];
C = eye(6);
D = 0;


% New Selection of Q and R:
Q = [1000 0 0 0 0 0;
    0 1000 0 0 0 0;
    0 0 1000 0 0 0;
    0 0 0 1000 0 0
    0 0 0 0 1000 0
    0 0 0 0 0 1000];
R = 0.001;

% Designing LQR Controller:
[K, ~, EigenValues] = lqr(A, B, Q, R);

% Setting up LQR system using built in state space function
LQR_Test_1 = ss(A-(B*K),B, C, D);

% Simulate and Plot Response
 figure()
 initial(LQR_Test_1, X_0)
 
 % Display Eigenvalues to test for stability.
 disp('Eigenvalues for Lyapunovs Indirect Method:')
 disp(EigenValues)

 %% Applying Tuned LQR Controller to Original Nonlinear System
 % Bring Down Tuning Params into This Section
 % Initial Selection of Q and R:
Q = [1000 0 0 0 0 0;
    0 1000 0 0 0 0;
    0 0 1000 0 0 0;
    0 0 0 1000 0 0
    0 0 0 0 1000 0
    0 0 0 0 0 1000];

% R = 0.001; % BAD R
R = 0.00001; % GOOD R

% Designing LQR Controller:
[K, P, EigenValues] = lqr(A, B, Q, R);

 % Simulating Function
 timevector = 0:0.1:200;
 X_0 = [0, 0, pi/4, 0, pi/4, 0];

 %Use ode45 to solve the state equation and provide the output as a
 %function of time and X_0
[t,state_history] = ode45(@(t,state)NL_system(state,t,K),timevector,X_0);

%Plot x
figure()
subplot(6,1,1);
plot(t,state_history(:,1),'k')
grid on;
ylabel('X (m)')
xlabel('Time (s)')

%Plot X_Dot
subplot(6,1,2);
plot(t,state_history(:,2),'k')
grid on;
ylabel('X Dot (m/s)')
xlabel('Time (s)')

%Plot theta1
subplot(6,1,3);
plot(t,state_history(:,3),'r')
grid on;
ylabel('Theta 1 (rad)')
xlabel('Time (s)')

%Plot theta1_dot
subplot(6,1,4);
plot(t,state_history(:,4),'r')
grid on;
ylabel('Theta 1 Dot (rad/s)')
xlabel('Time (s)')

%Plot theta2
subplot(6,1,5);
plot(t,state_history(:,5),'b')
grid on;
ylabel('Theta 2 (rad)')
xlabel('Time (s)')

%Plot theta2_dot
subplot(6,1,6);
plot(t,state_history(:,6),'b')
grid on;
ylabel('Theta 2 Dot (rad/s)')
xlabel('Time (s)')




%% Custom Built Functions
%Function that plugs in the state of the system and calculates the
% numeric NL system outputs based on the state.
 function Xdot = NL_system(s,t,K)
 MM = 1000;
 m1 = 100;
 m2 = 100;
 l1 = 20;
 l2 = 10;
 g = 9.81;
 F = -K*s;
 Xdot = [s(2);
     (1/(m1+m2+MM-m1*cos(s(3))^2-m2*cos(s(5))^2))*(F-m1*g*sin(s(3))*cos(s(3))-m1*l1*(s(4))^2*sin(s(3))-m2*g*sin(s(5))*cos(s(5))-m2*l2*(s(6))^2*sin(s(5)));
     s(4);
     (1/l1)*(((1/(m1+m2+MM-m1*cos(s(3))^2-m2*cos(s(5))^2))*(F-m1*g*sin(s(3))*cos(s(3))-m1*l1*(s(4))^2*sin(s(3))-m2*g*sin(s(5))*cos(s(5))-m2*l2*(s(6))^2*sin(s(5))))*cos(s(3)))-(g*sin(s(3)))
     s(6);
     (1/l2)*(((1/(m1+m2+MM-m1*cos(s(3))^2-m2*cos(s(5))^2))*(F-m1*g*sin(s(3))*cos(s(3))-m1*l1*(s(4))^2*sin(s(3))-m2*g*sin(s(5))*cos(s(5))-m2*l2*(s(6))^2*sin(s(5))))*cos(s(5)))-(g*sin(s(5)))];
 end

