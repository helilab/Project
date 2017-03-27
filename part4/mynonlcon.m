% Nonlinear constraints
function [c,ceq] = mynonlcon(x)
    N = 40;
    nx = 6;
    nu =2;
    beta  = 20;
    alfa = 0.2;
    lambda_t = 2*pi/3;
    c = -inf*ones(N*(nx+nu),1);
    ceq = zeros((nx+nu)*N,1);
    c(5:6:N*nx) = alfa *exp(-beta*(x(1:6:N*nx)- lambda_t).^2) - x(5:6:N*nx);   % Compute nonlinear inequalities at x.
end