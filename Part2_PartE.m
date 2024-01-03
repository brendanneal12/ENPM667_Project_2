%% Brendan Neal, Adam Lobo, and Hoang Pham
%% ENPM667 Project 2 (Final Project)
%% Part 2 Part E: Observability Conditions
clear
clc

syms MM m1 m2 l1 l2 g

%Comment these out for symbolic form or in for numerical values and form
%MM = 1000;
%m1 = 100;
%m2 = 100;
%l1 = 20;
%l2 = 10;
%g = 9.8;

%A matrix
A = [0, 1, 0, 0, 0, 0;
    0, 0, -m1*g/MM, 0, -m2*g/MM, 0;
    0, 0, 0, 1, 0, 0;
    0, 0, -(MM+m1)*g/(MM*l1), 0, -m2*g/(MM*l1), 0;
    0, 0, 0, 0, 0, 1;
    0, 0, -m1*g/(MM*l2), 0, -(MM+m2)*g/(MM*l2), 0];

%B matrix
B = [0; 1/MM; 0; 1/(MM*l1); 0; 1/(MM*l2)];

%C matrices (1 -> x;   2 -> o1 & o2;   3 -> x & o2;   4 -> x & o1 & o2)
C1 = [1, 0, 0, 0, 0, 0];
C2 = [0, 0, 1, 0, 1, 0];
C3 = [1, 0, 0, 0, 1, 0];
C4 = [1, 0, 1, 0, 1, 0];

%Observability matrices
OBSVBLT1 = [transpose(C1), transpose(A)*transpose(C1), transpose(A)^2*transpose(C1), transpose(A)^3*transpose(C1), transpose(A)^4*transpose(C1), transpose(A)^5*transpose(C1)];
OBSVBLT2 = [transpose(C2), transpose(A)*transpose(C2), transpose(A)^2*transpose(C2), transpose(A)^3*transpose(C2), transpose(A)^4*transpose(C2), transpose(A)^5*transpose(C2)];
OBSVBLT3 = [transpose(C3), transpose(A)*transpose(C3), transpose(A)^2*transpose(C3), transpose(A)^3*transpose(C3), transpose(A)^4*transpose(C3), transpose(A)^5*transpose(C3)];
OBSVBLT4 = [transpose(C4), transpose(A)*transpose(C4), transpose(A)^2*transpose(C4), transpose(A)^3*transpose(C4), transpose(A)^4*transpose(C4), transpose(A)^5*transpose(C4)];

%Simplification (comment out if using numerical values)
for r = 1:6

    for c = 1:6

        OBSVBLT1(r, c) = simplify(OBSVBLT1(r, c));
        OBSVBLT2(r, c) = simplify(OBSVBLT2(r, c));
        OBSVBLT3(r, c) = simplify(OBSVBLT3(r, c));
        OBSVBLT4(r, c) = simplify(OBSVBLT4(r, c));

    end

end

%Output controllability matrix (simplified)
OBSVBLT1
OBSVBLT2
OBSVBLT3
OBSVBLT4

%Rank of observability matrices: < 6 = not observable, = 6 = observable
rank1 = rank(OBSVBLT1)
rank2 = rank(OBSVBLT2)
rank3 = rank(OBSVBLT3)
rank4 = rank(OBSVBLT4)