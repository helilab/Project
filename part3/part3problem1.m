% Authors: Magne Hov, Audun Nytroe, Torje Digernes

% Run the init script
run('init07.m')
pleaseplot = 0;
% state vector^T = [lambda, r, p dot(p)]

% Time step
h = 0.25;
% Horizon
N = 100;
% states
nx = 4;
% inputs
nu = 1;

lambda_0 = pi;
lambda_f = 0;
x_0 = [lambda_0 0 0 0]';
x_f = [lambda_f 0 0 0]';


A_c = [0 1 0 0,
    0 0 -K_2 0,
    0 0 0 1,
    0 0 -K_1*K_pp -K_1*K_pd];

B_c = [0;
    0;
    0;
    K_1*K_pp];
C_c = [1 1 1 1]';

A_d = (eye(4) + A_c*h);
B_d = h*B_c;
C_d = C_c;
Q = zeros(4,4);
Q(1,1) = 1;
qq = [ 0.1 1 10 ];
zz = [];%zeros(N*(nx+nu), 1);
%%
for q = qq
%q = 0.1; % 0.1, 1, 10
    R = q;

    G = [kron(eye(N),Q) zeros( N*nx, N*nu );zeros( N*nx, N*nu )' kron(eye(N), R)];

Aeq = [ kron( eye(N), eye(nx) )+kron(diag(ones(1,N-1),-1),-A_d) kron( eye(N), -B_d)];
Beq = zeros((nx)*N,1);
size(Beq);
Beq(1:4) = A_d*x_0;

Ain = [];
Bin = [];

statelimit = [inf inf 30*pi/180 inf]';
inputlimit = [30*pi/180 ];
bound = [repmat(statelimit, N,1) ; repmat(inputlimit, N,1)];
lb = -bound;
ub = +bound;
z = quadprog( G, [],[],[],Aeq,Beq,lb,ub);
zz = [zz z];

%plot(z(1:4:N*nx))
end
%%
if pleaseplot ==true
    hold off
    figure(1);
    plot([0:h:h*N], [repmat(x_0(1,:),1,3);zz(1:4:N*nx,:)],'.')
    xlabel('time')
    legend('\lambda: q=0.1', '\lambda: q=1', '\lambda: q=10');
    title('Optimal open loop travel trajectory');
    FigHandle = figure(1);
    set(FigHandle, 'Position', [0, 600, 600, 275]);

    grid on
    figure( 2)
    stairs([0:h:h*(N-1)]' ,  zz(N*nx+1:1:end,:))

    title('Optimal open loop pitch reference trajectory');
    grid on
    xlabel('time')
    legend('p_c: q=0.1', 'p_c: q=1', 'p_c: q=10');
    FigHandle = figure(2);
    set(FigHandle, 'Position', [1000, 600, 600, 275]);
end

%%
optimalInput = [[0:h:35-h]' [repmat([ 0 ],20,1)  ; zz(nx*N+1:end,2)  ;   repmat([ 0 ],20,1)]]

optimalTrajectory = timeseries;
optimalTrajectory.time  = [0:h:35-h]';
optimalTrajectory.data = [zeros(4,20) reshape(zz(1:nx*N,2),[4, 100])+repmat([-pi 0 0 0]', 1,100)  repmat([-pi 0 0 0 ]', 1, 20)]';
%% Adding on the advanced Control Layer with LQR

Q_lqr = diag([1 0 0 0]);
R_lqr = 1;

% Calculating the optimal feedback gain K
[K, P, eigenvalues] = dlqr(A_d, B_d, Q_lqr, R_lqr);
 
K %= [0 0 0 0]













%