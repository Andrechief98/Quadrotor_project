function J = costFunction(X, U, e, data, Q_, R_, P_)

    Q = Q_*eye(3);
    R = R_*eye(3);
    P = P_*eye(3);
    
    p = data.PredictionHorizon;
    ref = data.References(1,:)';
    
    X1 = X(2:p+1,1);
    X2 = X(2:p+1,2);
    X3 = X(2:p+1,3);
    X_ = [X1'; X2'; X3'];
    % size(X_)
    
    U1 = U(1:p,1);
    U2 = U(1:p,2);
    U3 = U(1:p,3);
    U_ = [U1'; U2'; U3'];
    % size(U_)
    

    J = 0;
    for i = 1:(p-1)
        J = J+ (X_(:,i)-ref)'*Q*(X_(:,i)-ref) + U_(:,i)'*R*U_(:,i);
    end
    
    J = J + (X_(:,end)-ref)'*P*(X_(:,end)-ref);

end

