load('test.mat')

% map = logical(map);
% obs = logical(obs);


% Put walls and obstacles into map
map = walls | obs | goal;
obsT = logical(obs);                % All of the obstacles
goal = goal | goal;
walls = logical(walls);

mapI = zeros(size(map));
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
                xm = [xm j];
                ym = [ym i];
            end
        end
    end

 

[mapF,mapP] = mdp(map,mapI,500);

mapP(obsT) = -1;
mapP(walls) = -1;
mapP(goal) = 1;

%%

% % Calculate path from starting point (28,20)
% xP = [28];
% yP = [20];
% xc = 28;
% yc = 20;
% there = false;
% while(~there)
%    policy = mapP(yc,xc);
%    if(policy == 0)
%        yc = yc+1;
%    elseif (policy == 90)
%        xc = xc +1;
%    elseif (policy == 180)
%        yc = yc -1;
%    else
%        xc = xc -1;
%    end
%    
%    xP = [xP,xc];
%    yP = [yP,yc];
%    xc
%    yc 
%    policy
%    
%    if(mapP(yc,xc) == 1)
%        there = true;
%    end
%     
%     
% end


%%

figure(1); clf;
plot(xm,ym,'.');
hold on;
% plot(xP,yP);

axis([0 Mm 0 Nm]);
axis('square');

% Plot values
figure(2); clf;
bar3(mapF);

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