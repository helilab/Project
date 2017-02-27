% Authors: Magne Hov, Audun Nytr�, Torje Digernes
% woop: aa


% Run the init script
run('init.m')

% state vector^T = [lambda, r, p dot(p)]


%time step
h = 0.1;

A_c = [0 1 0 0,
    0 0 K_2 0,
    0 0 0 1,
    0 0 K_1*K_pp K_1*K_pd];

B_c = [0,
    0,
    0,
    K_1*K_pp];

A_d = (eye(4) + A_c*h);
B_d = h*B_c;
