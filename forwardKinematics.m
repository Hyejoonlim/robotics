%% Forward Kinematics
%Geometric Parameters
l1 = 40; % Link 1
l2 = 40; % Link 2
l3 = 5; % End effector

%Angular values with time
theta1 = deg2rad([ 90 0 60 0 90]);
theta2 = deg2rad([ 0 90 120 90 0]);
time = [ 0 2 4 8 10];

% Time setting
ts = 0; tf = 10; dt = 0.05;
tspan = [ts:dt:tf];

% Interpolating the values of theta1 and theta2
t1 = interp1(time, theta1, tspan, 'spline');
t2 = interp1(time, theta2, tspan, 'spline');

% Applying Forward Kinematics
x = l1*cos(t1) + l2*cos(t1+t2);
y = l1*sin(t1) + l2*sin(t1+t2);

%% Plot
figure(1)
grid on 
plot(tspan, x)
hold on
plot(tspan, y)
legend('x', 'y')

figure(2)
comet(x, y)
title('Desired Motion of the End Effector')
xlabel('x')
ylabel('y')

%% Animate
figure(3)
for i = 1:length(t1)
    T1 = t1(i); T2 = t2(i);
    
     %Coordinates
     x0 = 0; y0 = 0;
     x1 = l1*cos(T1); y1 = l1*sin(T1); 
     x2 = x1 + l2*cos(T1+T2); y2 = y1 + l2*sin(T1+T2);
     x3 = x2 - l3;  y3 = y2;
       
     % Plot
     plot([x0 x1], [y0 y1], [x1 x2], [y1 y2], [x2 x3], [y2 y3] ,'linewidth', 3)
     legend('Link 1', 'Link 2', 'End Effector')
     axis([-25 60 -5 85])
     pause(0.000001)
end