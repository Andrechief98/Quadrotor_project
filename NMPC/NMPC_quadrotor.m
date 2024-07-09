clear
clc
close all

rng(0) % For riproducibility

% SPECIFY NUMBER OF STATES, INPUT AND OUTPUT
nx = 3;
nu = 3;
ny = 3;

% CREATE THE NMPC OBJECT
nlmpcobj = nlmpc(nx, ny, nu);

% SPECIFY THE NUMBER OF PARAMETERS
% Parameters are passed to Prediction model, Jacobian function and Cost
% function, always. 
% We have to specify the number of parameters and then add such parameters
% as arguments of all the functions (specified as simple comma-separated
% list in all the function arguments)
nlmpcobj.Model.NumberOfParameters=3;



% PREDICTION MODEL AND JACOBIAN DEL PREDICTION MODEL
nlmpcobj.Model.StateFcn = "NMPC_model_dynamics"; % Prediction model

nlmpcobj.Jacobian.StateFcn  = "NMPC_model_jacobian"; %Jacobiano model (speed up the optimization but can it can be commented)


% SPECIFICATIONS OF THE NMPC
Ts = 0.2; % Sampling time
p = 25; % Prediction horizon
m=2; % Control horizon

nlmpcobj.Ts = Ts;
nlmpcobj.PredictionHorizon = p;
nlmpcobj.ControlHorizon = m;

% CONTROL INPUT CONSTRAINTS
nlmpcobj.MV = struct( ...
    Min={-10; -10; -10}, ...
    Max={10; 10; 10});
    % RateMin={-2;-2;-2}, ...
    % RateMax={2;2;2} ...



% CUSTOM COST FUNCTION
% In this way it is possible to use a custom cost function
nlmpcobj.Optimization.CustomCostFcn = "costFunction";
nlmpcobj.Optimization.ReplaceStandardCost = true;


% SPECIFY THE INITIAL CONDITION
x0 = [0;0;0];

nloptions = nlmpcmoveopt; % Needed to pass external parameters (weights) to the cost function

% TESTING DELL'NMPC
% validateFcns(nlmpcobj,rand(nx,1),rand(nu,1),[],{1, 1, 1, 1, 1, 1, 1, 1, 1, 1}); %help validateFcns


%% SIMULATION
% 
Tot_steps = 100; 



% MV last value is part of the controller state
lastMV = [0;0;0];

% Store states for plotting purposes
xHistory = x0';
uHistory = lastMV';

% Reference trajectory generation
radius = linspace(0,5,Tot_steps);
x = zeros(1,Tot_steps);
y = zeros(1,Tot_steps);
angles = linspace(0,4*pi,Tot_steps);

for k = 1:Tot_steps
    x(1,k) = radius(k)*cos(angles(k));
    y(1,k) = radius(k)*sin(angles(k));
end
 z = linspace(0,20,Tot_steps);

reference = [
    x;
    y;
    z
    ];

mpcverbosity("off")


% Simulation loop
for k = 1:Tot_steps
    k
    % Set references for previewing
    t = linspace(k*Ts, (k+p-1)*Ts,p);
    ref = reference(:,k)';

    % Compute control move with reference previewing
    xk = xHistory(k,:);
    
    nloptions.Parameters = {10, 5, 100};

    tic
    [uk,nloptions,info] = nlmpcmove(nlmpcobj,xk,lastMV,ref,[],nloptions);
    toc
    
    % Store control move
    % uHistory(k+1,:) = uk';
    lastMV = uk;

    % Simulate quadrotor for the next control interval (MVs = uk) 
    ODEFUN = @(t,xk) NMPC_model_dynamics(xk,uk);
    [TOUT,XOUT] = ode45(ODEFUN,[0 Ts], xHistory(k,:)');

    % Update robot state
    xHistory(k+1,:) = XOUT(end,:);
    uHistory(k,:) = lastMV;

end

xHistory = xHistory';

figure()
plot3(reference(1,:),reference(2,:),reference(3,:))
grid on
hold on
plot3(xHistory(1,2:end), xHistory(2,2:end), xHistory(3,2:end),"Color","r")
xlim([-10,10]);
ylim([-10,10]);
zlim([0,20]);
legend("Reference", "NMPC")


