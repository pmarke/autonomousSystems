clear all;
close all;
load('midterm_data.mat')

P0 = eye(3);     % initial covariance
X0 = X_tr(:,1);  % Initial estimate state
% X0 = zeros(3,1);
Ts = 0.1;        % Time step
eif = EIF(X0,P0);
ra = RobotAnimation(m,X_tr);

for ii=1:length(t)
%     u = [v(ii);om(ii)]; % Get input
    u = [v_c(ii);om_c(ii)]; % Get input
    eif.Predict(Ts,u);  % Predict
    
    % Update for each measurement
    for jj=1:length(m)
        ell = m(:,jj);
        r = range_tr(ii,jj);
        phi = bearing_tr(ii,jj);
        eif.Update(r,phi,ell)
    end
%     
    ra.Update(eif.mu,X_tr(:,ii),range_tr(ii,:),bearing_tr(ii,:));
    eif.UpdateHistory();
%     pause(0.05);
    
end

