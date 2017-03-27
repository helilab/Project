% Authors: Magne Hov, Audun Nytroe, Torje Digernes

% Run the init script
run('init05.m')

%% Parameters
% Time step
h = 0.25;
% Horizon
N = 40;
% Horizon period
T = N*h;
% Run time, horizon period + inital 5s and final 5s
T_run = T + 10;
% states
nx = 6;
% inputs
nu = 2;

lambda_0 = pi;
lambda_f = 0;
lambda_t = 2*pi/3;

x_0 = [lambda_0 0 0 0 0 0]';
x_f = [lambda_f 0 0 0 0 0]';

%% System
% Continuous system
A_c = [0 1 0 0 0 0,
    0 0 -K_2 0 0 0,
    0 0 0 1 0 0,
    0 0 -K_1*K_pp -K_1*K_pd 0 0,
    0 0 0 0 0 1,
    0 0 0 0 -K_3*K_ep -K_3*K_ed];

B_c = [0 0;
    0 0;
    0 0;
    K_1*K_pp 0;
    0 0;
    0 K_3*K_ep];
% Discrete system
A_d = (eye(nx) + A_c*h);
B_d = h*B_c;

%% MPC Optimization
% Linear equality constrains
Aeq = [ kron( eye(N), eye(nx) )+kron(diag(ones(1,N-1),-1),-A_d) kron( eye(N), -B_d)];
Beq = zeros((nx)*N,1);
size(Beq);
Beq(1:nx) = A_d*x_0;

% Linear inequality constraints 
Ain = [];
Bin = [];

% Boundaries
statelimit = [inf inf 30*pi/180 inf inf inf]';
inputlimit = [30*pi/180 inf]';
bound = [repmat(statelimit, N,1) ; repmat(inputlimit, N,1)];
lb = -bound;
ub = +bound;

% Creating object funciton cost matrix for QP problem
q_1 = 1;
q_2 = 1;

Q = diag([1 0 0 0 0 0]);
R = diag([q_1 q_2]);
costMatrix = [Q zeros(nx,nu); zeros(nu,nx) R ];
H = kron(eye(N), costMatrix);
% Creating object function for NLP problem
nonlinCost = diag([1,0,q_1,0,q_2,0]);
objectFunction = @(z) z(1);

% Solve
z = quadprog(H,[],[],[],Aeq,Beq,lb,ub);
%z = fmincon(objectFunction,z,Ain,Bin,Aeq,Beq,lb,ub,@mynonlcon);


%% Create input vectors for Simulink
optimalInput        = timeseries;
optimalInput.time   = [0:h:T_run-h]';
pitchReferenceData = [repmat([ 0 ],5/h,1); z(nx*N+1:2:end);  repmat([ 0 ],5/h,1)];
elevationReferenceData = [repmat([ 0 ],5/h,1); z(nx*N+2:2:end);  repmat([ 0 ],5/h,1)];
optimalInput.data  = [pitchReferenceData elevationReferenceData];
    
optimalTrajectory       = timeseries;
optimalTrajectory.time  = [0:h:T_run-h]';
optimalTrajectory.data  = [zeros(nx,5/h) reshape(z(1:nx*N),[nx, N])+repmat([-pi 0 0 0 0 0]',1,N)  repmat([-pi 0 0 0 0 0]', 1, 5/h)]';

%% Calculate deviation feedback gain K with dLQR
Q_lqr = diag([1 0 0 0 1 0]); % Penalize deviations in travel and elevation
R_lqr = diag([1 1]);

[K, P, eigenvalues] = dlqr(A_d, B_d, Q_lqr, R_lqr);

% Plot Input vectors and predicted states
shouldPlot = true;
if shouldPlot
   figure(1);
   plot(optimalInput);
   grid on;
   title('Optimal input references');
   legend('Optimal pitch reference', 'Optimal elevation reference');
   xlabel('Time [s]');
   ylabel('Angle [radians]');
   
   figure(2);
   plot(optimalTrajectory.time, [optimalTrajectory.data(:,1) optimalTrajectory.data(:,3) optimalTrajectory.data(:,5)]);
   grid on;
   title('Optimal trajectories');
   legend('Open loop travel trajectory', 'Open loop pitch trajectory', 'Open loop elevation trajectory');
   xlabel('Time [s]');
   ylabel('Angle [radians]');
   
   figure(3); % travel-elevation plot
   plot(optimalTrajectory.data(:,1), optimalTrajectory.data(:,5));
   grid on;
   title('Travel - Elevation trajectory');
   xlabel('Travel [radians]');
   ylabel('Elevation [radians]');
   
end
