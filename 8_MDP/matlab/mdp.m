function [mapF,mapP] = mdp(map,mapI,iterations)

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
mapP = zeros(map_size);    % Policy map;



for ii = 1:iterations
    for rr = 2:map_size(1)-1
        for cc = 2:map_size(2)-1
            if (~map(rr,cc))      % If not an obstacle, wall, or end goal

                % Get the values of the cells around the cell of interest
                up = mapF(rr+1,cc);
                down = mapF(rr-1,cc);
                right = mapF(rr,cc+1);
                left = mapF(rr,cc-1);

                % Calculate the cost for each action
                Vn = up*pf + right*pr + left*pl;
                Vs = down*pf + left*pr + right*pl;
                Ve = right*pf + down*pr + up*pl;
                Vw = left*pf + up*pr + down*pl;

                [V,P] = max([Vn,Ve,Vs,Vw]+ R) ;
    %             P = argmax([Vn,Vs,Ve,Vw]);

                mapF(rr,cc) = gamma*V;
                mapP(rr,cc) = (P-1)*90;
            end
        end
    end
end




end