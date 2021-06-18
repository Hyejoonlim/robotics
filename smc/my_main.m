%% System State-Space Model
A = [ 0 1; -1 -2];  % System matrix
B = [ 0; 1];         % Control matrix
C = eye(2);         % Output (sensor) matrix
D = zeros(2,1);     % Feed-forward Matrix

%% Time setting
ts = 0; tf = 10;dt = 0.001;   

%% Initial Conditions
ini = [2;2];    %initial condition 1
% ini = [2; 3]; %initial condition 2

%% Control gains
c = 1;           % Slope of Sliding line
k = 1;           % Control gains
beta_gain =1;    % Another approach beta gain
dist_gain = 10;  % Disturbance gain

%% ODE Solver using simulink 
simout = sim('ODE_Fucn1.slx');
t = simout.tout; x = simout.x; u = simout.u; s = simout.s;

%% Another Approach , gain [beta1, beta2]
simout = sim('ODE_Fucn2.slx');
t2 = simout.tout; x2 = simout.x; u2 = simout.u; s2 = simout.s;

%% Plotting 
sl = [-3, -2, -1, 0, 1, 2, 3];
plot(sl, -c*sl, 'r-')
hold on

%% Plotting 
sl = [-3, -2, -1, 0, 1, 2, 3];
plot(sl, -c*sl, 'r-')
hold on
comet(x2(:,1), x2(:,2))
hold off
legend('Sliding Line', 'System Response')
