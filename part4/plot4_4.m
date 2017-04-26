load part3problem2states
figure(1);
plot(statesout(1,:),statesout(2:2:7,:));
grid on;
title('Helicopter open loop trajectories');
legend('Helicopter travel trajectory', 'Helicopter pitch trajectory', 'Helicopter elevation trajectory');
xlabel('Time [s]');
ylabel('Angle [radians]');



figure(2);


load correctedByLQR

beta_m  = 20;
alfa = 0.2;
lambda_t = 2*pi/3;
options = optimset('MaxFunEvals',100000);
berg =@(lambda_k)( alfa * exp( - beta  * (lambda_k -lambda_t).^2)    )

plot(-statesout(2,5/0.002:end-(2.5/0.002)),[statesout(6,5/0.002:end-(2.5/0.002)) ;correctedByLQR(3,5/0.002:end-(2.5/0.002)) ]);
%plot(-statesout(2,5/0.002:end-(2.5/0.002)),[correctedByLQR(2:3,5/0.002:end-(2.5/0.002)) ]);
hold on
plot([0:0.01:pi], berg(+pi/3+[0:0.01:pi]));
%plot(-statesout(2,5/0.002:end-(2.5/0.002)),statesout(6,5/0.002:end-(2.5/0.002)));
hold off
grid on;
title('Helicopter Open Loop, Travel - Elevation trajectory');
xlabel('Travel [radians]');
ylabel('Elevation [radians]');
legend('Trajectory in space', 'Gråkallen');