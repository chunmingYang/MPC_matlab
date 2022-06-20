clc
clear all 
close all

%% MPC parameters
ts = 0.01;
T = 5;               % receding horizon can be tuned
iter = 200;          % iteration times, thus we have total_time = 200*0.01
n = 2;               % number of states (x, y)
m = 2;                % number of inputs/controls (v, omega)
Q = diag([2000 1000]);  % this can be tuned
R = zeros(m);          % inputs cost matrix since we don't have the target input trajectory to optimize then we set zero

%% initial guess for integrate problem

% states initialization
x = 0.1;
y = 0;
X = [x; y];
theta = 0;

% states reference initialization
xref = 0;
yref = 0;
Xref = [xref; yref];

% inputs/controls initialization
U = zeros(m, T); % in this case, u_1 is linear velocity, u_2 is the angular velocity

%% cubic trajectory generation
t0 = 0;
tf = ts*iter;
y0 = x;      % position start position
yf = 1.1;    % position end position

a0 = (yf*t0*t0*(3*tf-t0) + y0*tf*tf*(tf-3*t0))/((tf-t0)*(tf-t0)*(tf-t0));
a1 = 6*t0*tf*(y0-yf)/((tf-t0)*(tf-t0)*(tf-t0));
a2 = 3*(t0+tf)*(yf-y0)/((tf-t0)*(tf-t0)*(tf-t0));
a3 = 2*(y0-yf)/((tf-t0)*(tf-t0)*(tf-t0));

%% data saving
xhis = [];
yhis = [];
xrefhis = [];
yrefhis = [];

%% MPC main loop
for i = 1 : iter
    Aieq = [];   % since no ineuqality constraint
    bieq = [];   % since no inequality constraint
    Aeq = [];    % since no equality constraint
    beq = [];    % since no equality constraint
    lb = -10*ones(m, T);
    ub = 10*ones(m, T);

    % cubic trajectory
    xref = a0 + a1*(i*ts) + a2*(i*ts)*(i*ts) + a3*(i*ts)*(i*ts)*(i*ts);
    yref = a1 + 2*a2*(i*ts) + 3*a3*(i*ts)*(i*ts);
    Xref = [xref; yref];
    xrefhis(i) = xref;
    yrefhis(i) = yref;

    u = fmincon(@(U)cost_function(X,U,Xref,theta,ts,Q,R),U,Aieq,bieq,Aeq,beq,lb,ub);
    U = u; % this is for warm start
    u = u(:,1);
    theta = theta + ts*u(2);
    X = X + ts*u(1)*[cos(theta); sin(theta)];
    xhis(i) = X(1);
    yhis(i) = X(2);
end

%% plots interpretation
t = [];
for i = 1:length(xhis)
    t(i) = i*ts;
end

subplot(1,2,1)
plot(t, xhis); hold on;
plot(t, xrefhis)
legend("actual", "reference")

subplot(1,2,2)
plot(t, yhis); hold on;
plot(t, yrefhis)
legend("actual", "reference")