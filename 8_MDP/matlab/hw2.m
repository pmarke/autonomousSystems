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
obs1(92:95,6:100) = true;
obs1(6:92,6:10)=true;
obs1(6:10,14:96)=true;
% obs1(10:20,60:65) = true;

% Another obstacle
obs2(14:88,14:18) = true;
obs2(84:88,18:48) = true;
obs2(14:18,18:40) = true;
obs2(10:26,44:48) = true;

% Another obstacle
obs3(14:92,92:96) = true;
obs3(70:80,50:75) = true;

% The goal states
goal(96:99,96:99) = true;

% Put walls and obstacles into map
map = walls | obs1 | obs2 | obs3 | goal;
obsT = obs1 | obs2 | obs3;                % All of the obstacles
goal = goal | goal;
walls = logical(walls);

mapI = zeros(Np,Np);
mapI(obsT) = -500;
mapI(walls) = -100;
mapI(goal) = 1000;

% Plot map
% Sort through the cells to determine the x-y locations of occupied cells
[Mm,Nm] = size(map);
xm = [];
ym = [];
    for i = 1:Mm
        for j = 1:Nm
            if map(i,j)
                xm = [xm j];
                ym = [ym i];
            end
        end
    end

 

[mapF,mapP] = mdp(map,mapI,5000);
% mapF = mapI;

mapP(obsT) = -1;
mapP(walls) = -1;
mapP(goal) = 1;

%%

% Calculate path from starting point (98,86)
xP = [98];
yP = [86];
xc = xP;
yc = yP;
there = false;
while(~there)
   policy = mapP(yc,xc);
   if(policy == 0)
       yc = yc+1;
   elseif (policy == 90)
       xc = xc +1;
   elseif (policy == 180)
       yc = yc -1;
   else
       xc = xc -1;
   end
   
   xP = [xP,xc];
   yP = [yP,yc];
   xc
   yc 
   policy
   
   if(mapP(yc,xc) == 1)
       there = true;
   end
    
    
end


%%

figure(1); clf;
plot(xm,ym,'.');
hold on;
plot(xP,yP);

axis([0 Np+1 0 Np+1]);
axis('square');

% Plot values
figure(2); clf;
bar3(mapF);
% 
% Plot policy
figure(3);clf;
hold on;
x0 = [];
y0 = [];
angle = [];
length = 1;

map_size = size(mapI);

for rr = 2:map_size(1)-1
    for cc = 2:map_size(2)-1
        if (~map(rr,cc))      % If not an obstacle, wall, or end goal
            x0 = [x0;cc];
            y0 = [y0;rr];
            angle = [angle;mapP(rr,cc)*pi/180];
        end
    end
end

draw_arrow(x0,y0,length,angle);
axis([0,100,0,100])



