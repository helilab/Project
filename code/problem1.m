% Authors: Magne Hov, Audun Nytr�, Torje Digernes


% Run the init script
run('init.m')

A_c = [0 1 0 0,
    0 0 K_2 0,
    0 0 0 1,
    0 0 K_1*K_pp K_1*K_pd];

exp(A_c)