%% Brendan Neal, Adam Lobo, and Hoang Pham
%% ENPM667 Project 2 (Final Project)

%% Part 1 Part C: Symbolic Controllability Conditions
clc

syms MM m1 m2 l1 l2 g

%Comment these out for symbolic form or in for numerical values and form
% MM = 20
% m1 = 5
% m2 = 4
% l1 = 1
% l2 = 1
% g = -9.8

%A matrix
A = [0, 1, 0, 0, 0, 0;
    0, 0, -m1*g/MM, 0, -m2*g/MM, 0;
    0, 0, 0, 1, 0, 0;
    0, 0, -(MM+m1)*g/(MM*l1), 0, -m2*g/(MM*l1), 0;
    0, 0, 0, 0, 0, 1;
    0, 0, -m1*g/(MM*l2), 0, -(MM+m2)*g/(MM*l2), 0];

%B matrix
B = [0; 1/MM; 0; 1/(MM*l1); 0; 1/(MM*l2)];

%Controllability Matrix (not simplified)
CNTR = [B, A*B, A^2*B, A^3*B, A^4*B, A^5*B];

%Simplification
for r = 1:6

    for c = 1:6

        CNTR(r, c) = simplify(CNTR(r, c));

    end

end

%Output controllability matrix (simplified)
CNTR

%Rank of controllability matrix: < 6 = not controllable, = 6 = controllable
rank(CNTR)