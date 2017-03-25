load part4problem3states;
load part4problem3lqr;

modelsamples = lqrout(1,1:end-1);
optpath = -[ 0*ones(1,20) zz(1:4:nx*N,2)'-pi -pi*ones(1,20)];
%optpitchref = [ 0*ones(1,20) zz(nx*N+1:end,2)' 0*ones(1,20)]
extrapolatedPitchRef = interp1(0:h:35-h,optimalInput(:,2)',modelsamples-.248,'next');
modPitchRef = [lqrout(2,1:end-1)- interp1(0:h:35-h, optimalInput(:,2)', lqrout(1,1:end-1))];

figure(1);
plot(lqrout(1,1:end-1)',[modPitchRef;extrapolatedPitchRef])
grid
legend('Pitch reference after LQR correction','Pitch reference from quadprog');
xlabel('Time [seconds]');
ylabel('Angul [radians]');
title ('Pitch references');

figure(2);
optimalPitch = [ 0*ones(1,20) zz(3:4:nx*N,2)' -0*ones(1,20)];
optimalExtendedPitch = interp1(0:h:35-h,optimalPitch,modelsamples-h,'next');
plot(lqrout(1,1:end-1)',[-statesout(4,1:end-1);optimalExtendedPitch] );
grid
legend('Pitch from helicopter', 'Pitch from quadrog');
xlabel('Time [seconds]');
ylabel('Angle [radians]');
title('Pitch');

figure(3);
optimalTravel = [ pi*ones(1,20) zz(1:4:nx*N,2)' 0*ones(1,20)];
optimalExtendedTravel= interp1(0:h:35-h,optimalTravel,modelsamples-h,'next');
plot(statesout(1,1:end-1)',[ pi-statesout(2,1:end-1); optimalExtendedTravel]' );
grid
legend('Travel from LQR corrected model','Travel from quadprog');
xlabel('Time [seconds]');
ylabel('Angle [radians]');
title ('Travel reference and travel');