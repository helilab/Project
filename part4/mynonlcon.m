% Nonlinear constraints
function [c,ceq] = mynonlcon(x)
    global N nx alfa beta ourlambda;
    %c = -inf*ones(N*(nx+nu),1);
    ceq = [];%zeros((nx+nu)*N,1);
    %c(5:6:N*nx) = alfa *exp(-beta*(x(1:6:N*nx)- lambda_t).^2 ) - x(5:6:N*nx);   % Compute nonlinear inequalities at x.
    c = alfa *exp(-beta*(x(1:6:N*nx) - ourlambda).^2 ) - x(5:6:N*nx);   % Compute nonlinear inequalities at x.
    
end