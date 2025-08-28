mdl = 'GUAM';

load_system(mdl);
set_param(mdl, 'FastRestart', 'on');

%% Define the details of the Q,R matrix
nVar = 17; % Q : 6, R : 11 respect to diagonal component
ub = [2000 2000 2000 2000 2000 2000 2000 2000 2000 2000 2000 2000 2000 2000 2000 2000 2000]; % upper bound
lb = [zeros(1,6), 1e-3*ones(1,11)]; % lower bound, R component에는 0이 들어가면 안됨. 따라서 하한선 1*e-3으로 정해서 리카티해 구할 수 있도록 함.
fobj = @PSO_tunning; % fitness(objective) function name

% nVar_lat = 16; % Q : 6, R : 10 respect to diagonal component
% ub_lat = [2000 2000 2000 2000 2000 2000 2000 2000 2000 2000 2000 2000 2000 2000 2000 2000]; % upper bound
% lb_lat = [0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0]; % lower bound

%% Define PSO inital parameters

noP = 15; % number of particles
maxIter = 50; % maximum iteration number
wMax = 1; % inertia weight max
wMin = 0.1; % inertia weight min
c1 = 2;
c2 = 2;
vMax = (ub - lb) .* 0.2; 
vMin  = -vMax;

% The PSO algorithm 

% Initialize the particles 
for k = 1 : noP
    Swarm.Particles(k).X = (ub-lb) .* rand(1,nVar) + lb; 
    Swarm.Particles(k).V = zeros(1, nVar); 
    Swarm.Particles(k).PBEST.X = zeros(1,nVar); 
    Swarm.Particles(k).PBEST.O = inf; 
    
    Swarm.GBEST.X = zeros(1,nVar);
    Swarm.GBEST.O = inf;
end

cgCurve = nan(1, maxIter); % 사이즈 미리 초기화해서 시뮬 시간 축소
% Main loop
for t = 1 : maxIter
    
    % Calcualte the objective value
    for k = 1 : noP
        currentX = Swarm.Particles(k).X;
        Swarm.Particles(k).O = fobj(currentX);
        
        % Update the PBEST
        if Swarm.Particles(k).O < Swarm.Particles(k).PBEST.O 
            Swarm.Particles(k).PBEST.X = currentX;
            Swarm.Particles(k).PBEST.O = Swarm.Particles(k).O;
        end
        
        % Update the GBEST
        if Swarm.Particles(k).O < Swarm.GBEST.O
            Swarm.GBEST.X = currentX;
            Swarm.GBEST.O = Swarm.Particles(k).O;
        end
    end
    
    % Update the X and V vectors 
    w = wMax - t .* ((wMax - wMin) / maxIter);
    
    for k = 1 : noP
        r1 = rand(1, nVar);
        r2 = rand(1, nVar);
        Swarm.Particles(k).V = w .* Swarm.Particles(k).V + c1 .* r1 .* (Swarm.Particles(k).PBEST.X - Swarm.Particles(k).X) ...
                                                                                     + c2 .* r2 .* (Swarm.GBEST.X - Swarm.Particles(k).X);
                                                                                 
        
        % Check velocities 
        index1 = find(Swarm.Particles(k).V > vMax);
        index2 = find(Swarm.Particles(k).V < vMin);
        
        Swarm.Particles(k).V(index1) = vMax(index1);
        Swarm.Particles(k).V(index2) = vMin(index2);
        
        Swarm.Particles(k).X = Swarm.Particles(k).X + Swarm.Particles(k).V;
        
        % Check positions 
        index1 = find(Swarm.Particles(k).X > ub);
        index2 = find(Swarm.Particles(k).X < lb);
        
        Swarm.Particles(k).X(index1) = ub(index1);
        Swarm.Particles(k).X(index2) = lb(index2);
        
    end

    fprintf('Iteration# %d   GBEST = %.6g\n', t, Swarm.GBEST.O);
    cgCurve(t) = Swarm.GBEST.O;
end
 
semilogy(cgCurve);
xlabel('Iteration#')
ylabel('Weight')