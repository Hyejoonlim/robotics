global m g c k
m=1; g=9.81;

%% Time Setting
ts=0;           % Start Time
tf=10;          % Finished Time
dt=0.001;       % Sampling Time
tspan=[ts:dt:tf];

%% Initial Condition
x0=[0;0;1; 0];          % [x1=r x2=rdot x3=theta x4=thetadot]

%% Sliding Mode Controller
c=[1;1];        % Slope of Sliding Line (Sliding Surface), c(1)>0, c(2)>0
k=5;            % Switching Control Gain, k>0

%% ODE function
[t,x]=ode45(@odefcn,tspan,x0);

%% Desired Trajectory
xd=[sin(2*t) 2*cos(2*t) 2*sin(t) 2*cos(t)];
xddot=[2*cos(2*t) -4*sin(2*t) 2*cos(t) -2*sin(t)];

%% Tracking Error (Error=Desired State-Current State)
e=[xd(:,1)-x(:,1) xd(:,2)-x(:,2) xd(:,3)-x(:,3) xd(:,4)-x(:,4)];

%% Sliding Surface (s=c*error+error dot)
s(:,1)=[c(1) 1]*e(:,1:2)';
s(:,2)=[c(2) 1]*e(:,3:4)';

%% Controller
tau=m.*x(:,3).^2.*(c(1).*xd(:,2)-c(1).*x(:,2)+xddot(:,2)+2./x(:,3).*x(:,2).*x(:,4)+1./x(:,3)*g.*cos(x(:,1))+k.*sign(s(:,1)));
f=m*(2*c(2).*xd(:,4)-c(2).*x(:,4)+xddot(:,4)-x(:,3).*x(:,2).^2+g.*sin(x(:,1))+k.*sign(s(:,2)));

u=[tau f];

%% PLOTTER
sl=[-1:1];    % Sliding Line Range
figure(1);clf;      % Sketch Phase Portrait for Error
subplot(2,1,1); plot(sl,-c(1)*sl,'r-'); hold on; comet(e(:,1),e(:,2));
xlabel('e_1');ylabel('e_2'); hold off
subplot(2,1,2); plot(sl,-c(2)*sl,'r-'); hold on; comet(e(:,3),e(:,4));
xlabel('e_3');ylabel('e_4'); hold off
figure(2);clf;      % Plot Time Response for s
subplot(2,1,1); plot(t,s(:,1));
xlabel('Time');ylabel('s_1[t]');
subplot(2,1,2); plot(t,s(:,2));
xlabel('Time');ylabel('s_2[t]');
figure(3);clf;      % Plot Time Response for u
subplot(2,1,1); plot(t,u(:,1));
xlabel('Time');ylabel('tau[t]');
subplot(2,1,2); plot(t,u(:,2));
xlabel('Time');ylabel('f[t]');

figure(4);clf; hold on; % Plot Regulating Time
xlabel('Time');ylabel('Tracking Error');
h1 = animatedline('color','b','LineWidth',2);
h2 = animatedline('color','r','LineWidth',2);
for i = 1:100:length(t)
    addpoints(h1,t(i,1),e(i,1));
    drawnow;
    addpoints(h2,t(i,1),e(i,3));
    drawnow;
    legend('theta', 'radii');
end
hold off

figure(5);clf; hold on; % Plot Tracking Time
plot(t, xd(:,1), t, xd(:,3))
xlabel('Time');ylabel('Tracking Performance');
h1 = animatedline('color','b','LineWidth',2);
h2 = animatedline('color','r','LineWidth',2);
for i = 1:100:length(t)
    addpoints(h1,t(i,1),x(i,1));
    drawnow
    addpoints(h2,t(i,1),x(i,3));
    drawnow
    legend('theta', 'radii');
end
hold off

figure(6);clf; % Sketch Motion of End Effector
xpd=xd(:,3).*cos(xd(:,1)); ypd=xd(:,3).*sin(xd(:,1));
xp=x(:,3).*cos(x(:,1)); yp=x(:,3).*sin(x(:,1));
plot(xpd,ypd); hold on;
xlabel('x');ylabel('y');axis([min(xp) max(xp) min(yp) max(yp)])
h1 = animatedline('color','r','MaximumNumPoints',2);
h2 = animatedline('color','r','marker','o','MaximumNumPoints',1);
h3 = animatedline('color','m');
for i = 1:50:length(t)
    addpoints(h1,[0 xp(i,1)],[0 yp(i,1)]);
    addpoints(h2,xp(i,1),yp(i,1));
    addpoints(h3,xp(i,1),yp(i,1));
    drawnow
    legend('Desired Trajectory','End Effector')
end

%% Function (Must be Located in Last row)
% If you want to build state space model without mathematical model, just quit!!
function xdot = odefcn(t,x)
global m g c k
% Desired trajectory
xd=[sin(2*t) 2*cos(2*t) 2*sin(t) 2*cos(t)];
xddot=[2*cos(2*t) -4*sin(2*t) 2*cos(t) -2*sin(t)];


e=[xd(1)-x(1) xd(2)-x(2) xd(3)-x(3) xd(4)-x(4)];
% e=xd-x;

% Sliding Surface
s(1)=[c(1) 1]*e(1:2)';
s(2)=[c(2) 1]*e(3:4)';

% Design Controller (Design as you wish!)
tau=m*x(3)^2*(c(1)*xd(2)-c(1)*x(2)+xddot(2)+2/x(3)*x(2)*x(4)+1/x(3)*g*cos(x(1))+k*sign(s(1)));
f=m*(2*c(2)*xd(4)-c(2)*x(4)+xddot(4)-x(3)*x(2)^2+g*sin(x(1))+k*sign(s(2)));

u=[tau f];

% State Space Equation
xdot=[x(2);
    -2/x(3)*x(2)*x(4)-1/x(3)*g*cos(x(1))+1/m/x(3)^2*tau;
    x(4);
    x(3)*x(2)^2-g*sin(x(1))+1/m*f;                      
    ];
end
