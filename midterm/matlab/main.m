%% Code Implementation of the EIF
% This portion of the midterm includes the basic EIF Implementation
% The results, the code for the EIF, and the code for the animation


%% Basic EIF Implementation
clear all;
close all;
load('midterm_data.mat')

P0 = eye(3);     % initial covariance
X0 = X_tr(:,1);  % Initial estimate state
% X0 = zeros(3,1);
Ts = 0.1;        % Time step
eif = EIF(X0,P0);
ra = RobotAnimation(m,X_tr);

for ii=2:length(t)
%     u = [v(ii);om(ii)]; % Get input
    u = [v_c(ii);om_c(ii)]; % Get input
    eif.Predict(Ts,u);  % Predict
    
%    Update for each measurement
    for jj=1:length(m)
        ell = m(:,jj);
        r = range_tr(ii,jj);
        phi = bearing_tr(ii,jj);
        eif.Update(r,phi,ell)
    end
    
    ra.Update(eif.mu,X_tr(:,ii),range_tr(ii,:),bearing_tr(ii,:));
    eif.UpdateHistory();
%     pause(0.05);
    
end

ra.drawEstimateTrack(eif.mu_history);

%% Results

% Variance and standard deviation of the error covariance diagonal entries
P_var = reshape([eif.P_history(1,1,:);eif.P_history(2,2,:);eif.P_history(3,3,:)],3,[]);
P_std = sqrt(P_var);

% Calculate errors
error_x = eif.mu_history(1,:)-X_tr(1,:);
error_y = eif.mu_history(2,:)-X_tr(2,:);
error_th = eif.mu_history(3,:)-X_tr(3,:);
ex = mean(norm(error_x));
ey = mean(norm(error_y));
eth = mean(norm(error_th));

% Plot values of the information vector versus time
f1 = figure(1);
clf;
plot(t,eif.zeta_history(1,:));
hold on
plot(t,eif.zeta_history(2,:));
plot(t,eif.zeta_history(3,:));
title("Information vector vs time");
legend('x component','y component','z component')

% Plot true and estimated
f2 = figure(2);
clf;
subplot(3,1,1)
plot(t,eif.mu_history(1,:),'b');
hold on
plot(t,X_tr(1,:),'g');
title(" x Position");
legend("Estimate","True");

subplot(3,1,2)
plot(t,eif.mu_history(2,:),'b');
hold on
plot(t,X_tr(2,:),'g');
title(" y Position");
legend("Estimate","True");

subplot(3,1,3)
plot(t,eif.mu_history(3,:),'b');
hold on
plot(t,X_tr(3,:),'g');
title("Heading");
legend("Estimate","True");



% Plot the error
f3 = figure(3);
clf;
subplot(3,1,1)
plot(t,error_x,'r');
hold on
plot(t,2*P_std(1,:),'b');
plot(t,-2*P_std(1,:),'b');
legend('error (m)','2*std');
title('x pos error')
ylim([-0.2,0.2])
xlabel("time (s)")

subplot(3,1,2)
plot(t,error_y,'r');
hold on
plot(t,2*P_std(2,:),'b');
plot(t,-2*P_std(2,:),'b');
legend('error (m)','2*std');
title('y pos error')
ylim([-0.2,0.2])
xlabel("time (s)")

subplot(3,1,3)
plot(t,error_th,'r');
hold on
plot(t,2*P_std(3,:),'b');
plot(t,-2*P_std(3,:),'b');
legend('error (rads)','2*std');
title('th error')
ylim([-0.2,0.2])
xlabel("time (s)")

flag_save_plots = false;
font_size = 10;

filepath = '/home/mark/projects/autonomousSystems/midterm/notes/images';



 

% Save the files
if (flag_save_plots)
    set(f1,'Units','Inches');
    pos = get(f1,'Position');
    set(f1,'PaperPositionMode','Auto','PaperUnits','Inches','PaperSize',[pos(3), pos(4)])
    print(f1,[filepath,'/InformationVector'],'-dpdf','-r0')

    set(f2,'Units','Inches');
    pos = get(f2,'Position');
    set(f2,'PaperPositionMode','Auto','PaperUnits','Inches','PaperSize',[pos(3), pos(4)])
    print(f2,[filepath,'/EstVSTrue'],'-dpdf','-r0')

    set(f3,'Units','Inches');
    pos = get(f3,'Position');
    set(f3,'PaperPositionMode','Auto','PaperUnits','Inches','PaperSize',[pos(3), pos(4)])
    print(f3,[filepath,'/error'],'-dpdf','-r0')

    set(ra.fig,'Units','Inches');
    pos = get(ra.fig,'Position');
    set(ra.fig,'PaperPositionMode','Auto','PaperUnits','Inches','PaperSize',[pos(3), pos(4)])
    print(ra.fig,[filepath,'/map'],'-dpdf','-r0')
end
