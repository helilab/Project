% Authors: Magne Hov, Audun Nytroe, Torje Digernes

% Run the init script
run('init05.m')

%% Parameters
% Time step
h = 0.25;
% Horizon
global N;
N = 80;
% Horizon period
T = N*h;
% Run time, horizon period + inital 5s and final 5s
T_run = T + 10;
% states
global nx;
nx = 6;
% inputs
global nu;
nu = 2;

lambda_0 = pi;
lambda_f = 0;
lambda_t = 2*pi/3;

x_0 = [lambda_0 0 0 0 0.001 0]';
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
Beq(1:nx) = A_d*x_0;

% Linear inequality constraints 
Ain = [];
Bin = [];

% Boundaries
upperstatelimit = [inf inf 30*pi/180 inf inf inf]';
lowerstatelimit = [-inf -inf -30*pi/180 -inf 0 -inf]';
upperinputlimit = [30*pi/180 30*pi/180]';
lowerinputlimit = [-30*pi/180 -30*pi/180]';
ub = [repmat(upperstatelimit, N,1) ; repmat(upperinputlimit, N,1)];
lb = [repmat(lowerstatelimit, N,1) ; repmat(lowerinputlimit, N,1)];

% Creating object funciton cost matrix for QP problem
global q_1 q_2;
q_1 = 1;
q_2 = 1;

Q = diag([1 0 0 0 0 0]);
R = diag([q_1 q_2]);
costMatrix = [Q zeros(nx,nu); zeros(nu,nx) R ];
H = kron(eye(N), costMatrix);
%   Modify matrices to include a setpoint above the mountain for quadprog
global beta;
beta = 20;
global alfa;
alfa = 0.2;
global ourlambda ;
ourlambda = 2*pi/3;

setPoint = [lambda_t 0 0 0 alfa 0]';
timeSteps = 50;
Beq_QP = [Beq; setPoint];
A_setpoint = diag([1 0 0 0 1 0]);
Aeq_QP = [Aeq; zeros(nx, (nx+nu)*N)];
Aeq_QP(nx*N+1:end, nx*timeSteps+1:nx*(timeSteps+1)) = A_setpoint;

% Solve
beta_m  = 20;
alfa = 0.2;
lambda_t = 2*pi/3;
options = optimset('MaxFunEvals',3000);
berg =@(lambda_k)( alfa * exp( - beta  * (lambda_k -lambda_t).^2)    )
z = quadprog(H,[],[],[],Aeq_QP,Beq_QP,lb,ub);
%z = fmincon(@myObjectFunction,z,Ain,Bin,Aeq,Beq,lb,ub,@mynonlcon,options);

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
    
   figure(1); % Plot inputs
   plot(optimalInput);
   grid on;
   title('Optimal input references');
   legend('Optimal pitch reference', 'Optimal elevation reference');
   xlabel('Time [s]');
   ylabel('Angle [radians]');
   
   figure(2); % Plot trajectories
   plot(optimalTrajectory.time, [optimalTrajectory.data(:,1) optimalTrajectory.data(:,3) optimalTrajectory.data(:,5)]);
   grid on;
   title('Optimal trajectories');
   legend('Open loop travel trajectory', 'Open loop pitch trajectory', 'Open loop elevation trajectory');
   xlabel('Time [s]');
   ylabel('Angle [radians]');
   
   figure(3); % Plot Travel/Elevation
   plot(-optimalTrajectory.data(:,1), [optimalTrajectory.data(:,5), berg(pi+optimalTrajectory.data(:,1))]);
    %plot(optimalTrajectory.data(:,1), [optimalTrajectory.data(:,5), berg(optimalTrajectory.data(:,1))]);
   grid on;
   title('Travel - Elevation trajectory');
   xlabel('Travel [radians]');
   ylabel('Elevation [radians]');
   
end
