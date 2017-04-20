% Compute cost at given vector
function cost = myObjectFunction(z)
    global N nx nu q_1 q_2;
    tempCost = 0;
    for i = 0:N-1
        % lambda
        tempCost = tempCost + z(i*nx + 1)^2;
        % pitch input
        tempCost = tempCost + q_1*z(N*nx + i*nu + 1)^2;
        % elevation input
        tempCost = tempCost + q_2*z(N*nx + i*nu + 2)^2;
    end
    cost = tempCost;
end