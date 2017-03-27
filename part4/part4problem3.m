% Authors: Magne Hov, Audun Nytroe, Torje Digernes

% Run the init script
run('init05.m')

%% Parameters
% Time step
h = 0.25;
% Horizon
N = 40;
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
statelimit = [inf inf 30*pi/180 inf]';
inputlimit = [30*pi/180 ];
bound = [repmat(statelimit, N,1) ; repmat(inputlimit, N,1)];
lb = -bound;
ub = +bound;

% Creating object function
q_1 = 0.5;q_2 = 0.5;
nonlinCost = diag([1,0,q_1,0,q_2,0]);
objectFunction = @(z) z(1);

% Solve
guess = quadprog(objectFunction,Ain,Bin,Aeq,Beq,lb,ub);
z = fmincon(objectFunction,[],Ain,Bin,Aeq,Beq,lb,ub,@mynonlcon);


%% Create input vectors for Simulink
optimalInput = [[0:h:35-h]' [repmat([ 0 ],20,1)  ; z(nx*N+1:end,2)  ;   repmat([ 0 ],20,1)]]

optimalTrajectory = timeseries;
optimalTrajectory.time  = [0:h:35-h]';
optimalTrajectory.data = [zeros(nx,20) reshape(z(1:nx*N,2),[nx, 100])+repmat([-pi 0 0 0]', 1,100)  repmat([-pi 0 0 0 ]', 1, 20)]';


%% Calculate deviation feedback gain K with dLQR
Q_lqr = diag([1 0 0 0 1 0]); % Penalize deviations in travel and elevation
R_lqr = 1;

[K, P, eigenvalues] = dlqr(A_d, B_d, Q_lqr, R_lqr);
 
