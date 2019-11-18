% MDP_hw_map.m
%
% Create a map for MDP path planning homework

clear all;

N = 100;
Np = 100 + 2;

map = zeros(Np,Np);        % map dimension

% Initialize walls and obstacle maps as empty
walls = zeros(Np,Np);
obs1 = zeros(Np,Np);
obs2 = zeros(Np,Np);
obs3 = zeros(Np,Np);
goal = zeros(Np,Np);

% Create exterior walls
walls(2,2:N) = true;
walls(2:N+1,2) = true;
walls(N+1,2:N+1) = true;
walls(2:N+1,N+1) = true;

% Create single obstacle
obs1(20:40,30:80) = true;
obs1(10:20,60:65) = true;

% Another obstacle
obs2(45:65,10:45) = true;

% Another obstacle
obs3(43:92,75:85) = true;
obs3(70:80,50:75) = true;

% The goal states
goal(75:80,96:98) = true;

% Put walls and obstacles into map
map = walls | obs1 | obs2 | obs3 | goal;
obsT = obs1 | obs2 | obs3;                % All of the obstacles
goal = goal | goal;
walls = logical(walls);

mapI = zeros(Np,Np);
mapI(obsT) = -5000;
mapI(walls) = -100;
mapI(goal) = 100000;

% Plot map
% Sort through the cells to determine the x-y locations of occupied cells
[Mm,Nm] = size(map);
xm = [];
ym = [];
    for i = 1:Mm
        for j = 1:Nm
            if map(i,j)
                xm = [xm i];
                ym = [ym j];
            end
        end
    end

figure(1); clf;
plot(xm,ym,'.');
axis([0 Np+1 0 Np+1]);
axis('square'); 

figure(2); clf;
bar3(mapI);


