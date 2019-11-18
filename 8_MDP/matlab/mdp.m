function mapF = mdp(map,mapI)

mapF = mapI;
gamma = 1;        % Discount factor;
R = -2;           % Nominal Cost;
pf = 0.8;         % Probability of going forward;
pr = 0.1;         % Probability of going right;
pl = 0.1;         % Probability of going left;

% Set outer edge values
mapF(:,1) = mapF(:,2);
mapF(:,end) = mapF(:,end-1);
mapF(1,:) = mapF(2,:);
mapF(end,:) = mapF(end-1,:);

map_size = size(mapI);





end