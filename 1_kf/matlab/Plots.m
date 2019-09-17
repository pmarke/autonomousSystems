log_K= fopen("/tmp/AS_KalmanFilter/k.log","r");
log_t= fopen("/tmp/AS_KalmanFilter/t.log","r");
log_x= fopen("/tmp/AS_KalmanFilter/x.log","r");
log_xh= fopen("/tmp/AS_KalmanFilter/xh.log","r");
log_u= fopen("/tmp/AS_KalmanFilter/u.log","r");
log_P= fopen("/tmp/AS_KalmanFilter/P.log","r");




K_array = reshape(fread(log_K,'float'),2,[]);
t_array = reshape(fread(log_t,'float'),1,[]);
x_array = reshape(fread(log_x,'float'),2,[]);
xh_array = reshape(fread(log_xh,'float'),2,[]);
u_array = reshape(fread(log_u,'float'),1,[]);
P_array = reshape(fread(log_P,'float'),2,2,[]);

std_pos = reshape(P_array(1,1,:),1,[]);
std_pos = sqrt(std_pos);
std_vel = reshape(P_array(2,2,:),1,[]);
std_vel = sqrt(std_vel);

e_array = x_array - xh_array;

%%

figure(1),clf
subplot(2,1,1)
plot(t_array,x_array(1,:))
hold on
plot(t_array,xh_array(1,:))
plot(t_array,e_array(1,:))
legend('True','Estimate','Error')
title("Position")
xlabel("Seconds")
ylabel("Meters")

subplot(2,1,2)
plot(t_array,x_array(2,:))
hold on
plot(t_array,xh_array(2,:),'*')
plot(t_array,e_array(2,:))
legend('True','Estimate','Error')
title("Velocity")
xlabel("Seconds")
ylabel("Meters/Sec")

figure(2),clf
subplot(2,1,1)
plot(t_array,2*std_pos,'b')
hold on
plot(t_array,-2*std_pos,'b')
plot(t_array,e_array(1,:),'r')
legend("2*std_dev","-2*std_dev","error")
title("Position")
xlabel("Seconds")
ylabel("Meters")

subplot(2,1,2)
plot(t_array,2*std_vel,'b')
hold on
plot(t_array,-2*std_vel,'b')
plot(t_array,e_array(2,:),'r')
legend("2*std dev","-2*std dev","error")
title("Velocity")
xlabel("Seconds")
ylabel("Meters/Sec")

figure(3),clf
subplot(2,1,1)
plot(t_array,K_array(1,:))
title("Position Kalman Gain")
xlabel("Seconds")

subplot(2,1,2)
plot(t_array,K_array(2,:))
title("Velocity Kalman Gain")
xlabel("Seconds")
