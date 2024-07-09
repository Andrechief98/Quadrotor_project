function [A,B] = NMPC_model_jacobian(x, u, Q_, R_, P_)

A = [0, 0, 0;
    0, 0, 0;
    0, 0, 0];

B = [1, 0, 0;
    0, 1, 0;
    0, 0, 1];
end

