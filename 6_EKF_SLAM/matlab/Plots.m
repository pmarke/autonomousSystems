clear all
close all

log_K= fopen("/tmp/AS_EKF/k.log","r");
log_t= fopen("/tmp/AS_EKF/t.log","r");
log_x= fopen("/tmp/AS_EKF/x.log","r");
log_xh= fopen("/tmp/AS_EKF/xh.log","r");
log_u= fopen("/tmp/AS_EKF/u.log","r");
log_P= fopen("/tmp/AS_EKF/P.log","r");
log_m= fopen("/tmp/AS_EKF/m.log","r");




K_array = reshape(fread(log_K,'float'),3,2,[]);
t_array = reshape(fread(log_t,'float'),1,[]);
x_array = reshape(fread(log_x,'float'),3,[]);
xh_array = reshape(fread(log_xh,'float'),3,[]);
u_array = reshape(fread(log_u,'float'),2,[]);
P_array = reshape(fread(log_P,'float'),3,3,[]);
num_meas = fread(log_m,1,'int');
m_pos = reshape(fread(log_m,2*num_meas,'float'),2,num_meas,[]);
m_array = reshape(fread(log_m,'float32'),2,num_meas,[]);
% m_array = fread(log_m,'float32');

std_px = reshape(P_array(1,1,:),1,[]);
std_px = sqrt(std_px);
std_py = reshape(P_array(2,2,:),1,[]);
std_py = sqrt(std_py);
std_w = reshape(P_array(3,3,:),1,[]);
std_w = sqrt(std_w);

e_array = x_array - xh_array;
dec = 10;

fclose(log_K);
fclose(log_t);
fclose(log_x);
fclose(log_xh);
fclose(log_u);
fclose(log_P);
fclose(log_m);

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

%%
animation_dec = 10;
for i = 1:animation_dec:length(m_array)
    
    animate(t_array(i),x_array(:,i+1),xh_array(:,i+1),num_meas,m_pos,m_array(:,:,i))
    pause(0.5)
    
end
