



% Value Function after action
V1 = [-100 100;...
      100  -50];
  
p_x1_x1_u3 = 0.2;           % bad outcome
p_x2_x1_u3 = 1-p_x1_x1_u3;
p_x2_x2_u3 = 0.2;
p_x1_x2_u3 = 1-p_x1_x1_u3;
p_z1_x1 = 0.7;             % Good outcome
p_z1_x2 = 1-p_z1_x1;
p_z2_x2 = 0.7;
p_z2_x1 = 1-p_z2_x2;
  
Z1 = [p_z1_x1 0;...
      0   p_z1_x2];
  
Z2 = [p_z2_x1 0;...
       0  p_z2_x2];
   
A = [p_x1_x1_u3 p_x1_x2_u3;...
     p_x2_x1_u3 p_x2_x2_u3];
  
V = V1;
  
T = 10;

num_pts = 10000;

for ii = 2:T

% Sense
V_Z1 = V*Z1;
V_Z2 = V*Z2;

n = length(V_Z1);
tmp1 = repmat(V_Z2,n,1);
tmp2 = zeros(n^2,2);
for jj=1:n
tmp2(jj*n-n+1:jj*n,:) = repmat(V_Z1(jj,:),n,1);
end

V_bar = tmp1+tmp2;

% Predict
tmp3 = V_bar*A;
V_p = [V1;tmp3-1];
    
V =prune(V_p,num_pts);
% V = V_p;
end



%% Plot results
figure(1),clf;
hold on
for ii = 1:length(V)
    plot([0,1],V(ii,:),'b');
end
ylim([0,100])


%% Trials

x = 1;   % Initial state
b = 0.6; % Initial belief


finished = false;

while(~finished)

% Take measurement
r = rand(1);       % generate random number
if (r > 0.7)       % observed correctly
    
    if (x ==1)
        b = 0.7*b/(0.4*b+0.3);
    else
        b = 0.3*b/(0.7-0.4*b);
    end

else
    
    if (x==1)
        b = 0.3*b/(0.4*b+0.3);
    else
        b = 0.7*b/(0.7-0.4*b);
    end
    
    
    
end

% Get policy
[v,policy] = max(V*[b;1-b]);

% Take action
if policy == 1         % action 1
    if x ==1
        reward = -100;
    else
        reward = 100;
    end
    finished = true;
elseif policy ==2      % action 2
   if x ==2
        reward = -50;
    else
        reward = 100;
    end
    finished = true; 
else                   % action 3
    
    r = rand(1);       % generate random number
    if (r > 0.8)       % apply action
        % update true state
        x = x+1;
        if x >2
            x = 1;
        end
    end
    
    % Update belief
    b = 0.8-0.6*b;

end

end
