%% Solving Robot Dynamic Equations
%% Parameters 
m1 = 1; m2 = 1; g = 9.81; 
a1 = 1; a2 = 1;

syms tau1 tau2 real
syms theta1 theta1_dot theta2 theta2_dot theta4 real
syms theta2d theta2d_dot theta4d theta4d_dot real
syms c1 c2 real

%% Formulation 
m11 = (m1+m2)*(a1^2) + m2*(a2^2) +2*m2*a1*a2*cos(theta2);
m12 = m2*(a2^2) + m2*a1*a2*cos(theta2);
m21 = m2*(a2^2) + m2*a1*a2*cos(theta2);
m22 =m2*(a2^2);

M = [ m11 m12; m21 m22];

v1 = -m2*a1*a2*(2*theta1_dot*theta2_dot + theta2_dot^2)*sin(theta2);
v2 = m2*a1*a2*(theta1_dot^2)*sin(theta2);

v = [ v1; v2];

g1 = (m1+m2)*g*a1*cos(theta1) + m2*g*cos(theta1+theta2);
g2 = m2*g*a2*cos(theta1+theta2);

g = [ g1; g2];

tau = [ tau1; tau2];

result = simplify (inv(M)*(tau - v - g));

%% Dynamic function for x2_dot and x4_dot
x2_dot = result(1)
x4_dot = result(2)

%% Finding Tau1 and Tau2
eqn1 = c1*(theta2d - theta2) + theta2d_dot - result(1) == 0;
eqn2 = c2*(theta4d - theta4) + theta4d_dot - result(2) == 0;

syms Tau1(theta1, theta1_dot, theta2, theta2_dot, theta4, theta2d, theta2d_dot, theta4d, theta4d_dot) 
syms Tau2(theta1, theta1_dot, theta2, theta2_dot, theta4, theta2d, theta2d_dot, theta4d, theta4d_dot) 

[A B] = equationsToMatrix([eqn1, eqn2], [tau1, tau2]);
Tau = linsolve(A, B);
Tau1 = Tau(1)
Tau2 = Tau(2)
