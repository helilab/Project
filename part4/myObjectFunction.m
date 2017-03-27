% Compute cost at given vector
function cost = mynonlcon(z)
    N = 40;
    nx = 6;
    nu =2;
    q1 = 1;
    q2 = 2;
    tempCost = 0;
    for i = 0:N-1
        % lambda
        tempCost = tempCost + z(i*nx + 1);
        % pitch input
        tempCost = tempCost + z(N*nx + i*nu + 1);
        % elevation input
        tempCost = tempCost + z(N*nx + i*nu + 2);
    end
    cost = tempCost;
end