load part3problem2states;
load part3problem2lqr;

modelsamples = lqrout(1,1:end-1);
optpath = -[ 0*ones(1,20) zz(1:4:nx*N,2)'-pi -pi*ones(1,20)];
%optpitchref = [ 0*ones(1,20) zz(nx*N+1:end,2)' 0*ones(1,20)]
extrapolatedPitchRef = interp1(0:h:35-h,optimalInput(:,2)',modelsamples-.248,'next');
modPitchRef = [lqrout(2,1:end-1)- interp1(0:h:35-h, optimalInput(:,2)', lqrout(1,1:end-1))];

figure(1);
plot(lqrout(1,1:end-1)',[-statesout(4,1:end-1); modPitchRef;extrapolatedPitchRef])
grid
legend('Pitch (Measured)','Pitch reference after LQR correction','Pitch reference from quadprog');
xlabel('Time [seconds]');
ylabel('Angle [radians]');
title ('Pitch references and pitch');

figure(2);

plot(statesout(1,1:125:end-1)',[ statesout(2,1:125:end-1); optpath]' );
grid
legend('Travel from LQR corrected model','Travel from quadprog','Location','southeast');
xlabel('Time [seconds]');
ylabel('Angle [radians]');
title ('Travel reference and travel');