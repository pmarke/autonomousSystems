function V = prune(V_p,num_pts)

p1 = linspace(0,1,num_pts);
pts = [p1;1-p1];

tmp = V_p*pts;
[~,index] = max(tmp);
index=unique(index);

V =V_p(index,:);


end