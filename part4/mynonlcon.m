% Nonlinear constraints
function [c,ceq] = mynonlcon(x)
    beta  = 20;
    alfa = 0.2;
    lambda_t = 2*pi/3;
    c = -inf*ones(N*(nx+nu),1);
    ceq = [];
    c(1:6:N*nx) = alfa *exp(-beta*(x(1:6:N*nx)- lambda_t)^2) - x(5:6:N*nx);   % Compute nonlinear inequalities at x.
end