function f = NMPC_model_dynamics(x, u, Q_, R_, P_)
% Tale funzione descrive il modello del sistema non lineare che viene
% usato come riferimento all'interno dell'NMPC. Tale sistema di equazioni
% differenziali viene risolto in seguito dalla funzione "solver" che andrà
% a discretizzare la funzione e ricavare il next state. Esso viene usato
% SOLO PER ESEGUIRE LE PREDIZIONI DURANTE IL PROCESSO DI OTTIMIZZAZIONE

    f = [
        u(1);
        u(2);
        u(3)
        ];

end

