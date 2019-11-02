clear all
close all

% log_K= fopen("/home/mark/projects/autonomousSystems/6_EKF_SLAM/log/k.log","r");
log_t= fopen("/home/mark/projects/autonomousSystems/6_EKF_SLAM/log/t.log","r");
log_x= fopen("/home/mark/projects/autonomousSystems/6_EKF_SLAM/log/x.log","r");
log_xh= fopen("/home/mark/projects/autonomousSystems/6_EKF_SLAM/log/xh.log","r");
log_u= fopen("/home/mark/projects/autonomousSystems/6_EKF_SLAM/log/u.log","r");
log_P= fopen("/home/mark/projects/autonomousSystems/6_EKF_SLAM/log/P.log","r");
log_m= fopen("/home/mark/projects/autonomousSystems/6_EKF_SLAM/log/m.log","r");
log_landmarks= fopen("/home/mark/projects/autonomousSystems/6_EKF_SLAM/log/landmarks.log","r");



% K_array = reshape(fread(log_K,'float'),3,2,[]);
num_landmarks =  fread(log_landmarks,1,'int');
landmarks = reshape(fread(log_landmarks,'float'),2,[]);
m_array = reshape(fread(log_m,'float32'),2,num_landmarks,[]);


t_array = reshape(fread(log_t,'float'),1,[]);
x_array = reshape(fread(log_x,'float'),3,[]);
xh_array = reshape(fread(log_xh,'float'),3+num_landmarks*2,[]);
u_array = reshape(fread(log_u,'float'),2,[]);
P_array = reshape(fread(log_P,'float32'),3+num_landmarks*2,[]);





std_px = sqrt(P_array);


tmp = repmat(reshape(landmarks,1,[])',1,length(t_array));
e_array = [x_array;tmp] - xh_array;
dec = 10;

% fclose(log_K);
fclose(log_t);
fclose(log_x);
fclose(log_xh);
fclose(log_u);
fclose(log_P);
fclose(log_m);

ra = RobotAnimation(landmarks,x_array);


%%
for ii = 2:length(t_array)
   ra.Update(xh_array(:,ii),x_array(:,ii),reshape(m_array(1,:,ii),1,[]),reshape(m_array(1,:,ii),1,[]));

   pause(0.01)
end
ra.drawEstimateTrack(xh_array(1:3,:))

%%

figure(5),clf;
hold on
for ii = 1:4
   plot(t_array,P_array(ii,:)); 
   min(P_array(ii,:))
end
legend('x','y','th','1','2','3','4','5','6')

figure(6),clf;
subplot(3,1,1)
plot(t_array,xh_array(1,:));
hold on;
plot(t_array,x_array(1,:));

subplot(3,1,2)
plot(t_array,xh_array(2,:));
hold on;
plot(t_array,x_array(2,:));

subplot(3,1,3)
plot(t_array,xh_array(3,:));
hold on;
plot(t_array,x_array(3,:));

%%

figure(1),clf
subplot(3,1,1)
plot(t_array(1:dec:end),x_array(1,1:dec:end))
hold on
plot(t_array(1:dec:end),xh_array(1,1:dec:end),'*')
legend('True','Estimate')
title("Position X")
xlabel("Seconds")
ylabel("Meters")

subplot(3,1,2)
plot(t_array(1:dec:end),x_array(2,1:dec:end))
hold on
plot(t_array(1:dec:end),xh_array(2,1:dec:end),'*')
legend('True','Estimate')
title("Position Y")
xlabel("Seconds")
ylabel("Meters")

subplot(3,1,3)
plot(t_array(1:dec:end),x_array(3,1:dec:end))
hold on
plot(t_array(1:dec:end),xh_array(3,1:dec:end),'*')
legend('True','Estimate')
title("Angular Velocity")
xlabel("Seconds")
ylabel("Radians/Sec")

figure(2),clf
subplot(3,1,1)
plot(t_array,2*std_px,'b')
hold on
plot(t_array,-2*std_px,'b')
plot(t_array,e_array(1,:),'r')
legend("2*std_dev","-2*std_dev","error")
title("Position X")
xlabel("Seconds")
ylabel("Meters")
ylim([-5*std_px(end),5*std_px(end)])

subplot(3,1,2)
plot(t_array,2*std_py,'b')
hold on
plot(t_array,-2*std_py,'b')
plot(t_array,e_array(2,:),'r')
legend("2*std_dev","-2*std_dev","error")
title("Position Y")
xlabel("Seconds")
ylabel("Meters")
ylim([-5*std_py(end),5*std_py(end)])

subplot(3,1,3)
plot(t_array,2*std_w,'b')
hold on
plot(t_array,-2*std_w,'b')
plot(t_array,e_array(3,:),'r')
legend("2*std dev","-2*std dev","error")
title("Angular Velocity")
xlabel("Seconds")
ylabel("Radians/Sec")
ylim([-5*std_w(end),5*std_w(end)])

[row,col] = size(K_array(:,:,1));
figure(3),clf;
title("Kalman Gains")
for r = 1:row
   for c = 1:col
        subplot(row,col,r*col+c-col)
        plot(t_array,reshape(K_array(r,c,:),1,[]))
   end
    
end

