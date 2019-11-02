clear all
close all

% log_K= fopen("/home/mark/projects/autonomousSystems/6_EKF_SLAM/log/k.log","r");
log_t= fopen("/home/mark/projects/autonomousSystems/6_EKF_SLAM/log/t.log","r");
log_x= fopen("/home/mark/projects/autonomousSystems/6_EKF_SLAM/log/x.log","r");
log_xh= fopen("/home/mark/projects/autonomousSystems/6_EKF_SLAM/log/xh.log","r");
log_u= fopen("/home/mark/projects/autonomousSystems/6_EKF_SLAM/log/u.log","r");
log_P= fopen("/home/mark/projects/autonomousSystems/6_EKF_SLAM/log/P.log","r");
log_P_full = fopen("/home/mark/projects/autonomousSystems/6_EKF_SLAM/log/P_full.log","r");
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
P_full = reshape(fread(log_P_full,'float32'),3+num_landmarks*2,3+num_landmarks*2);





std_p = sqrt(P_array);


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
   ra.Update(xh_array(:,ii),x_array(:,ii),std_p(:,ii),reshape(m_array(1,:,ii),1,[]),reshape(m_array(1,:,ii),1,[]));

   pause(0.01)
end
ra.drawEstimateTrack(xh_array(1:3,:))



%%

figure(1),clf
P_size = size(P_array);
dim = ceil(sqrt(P_size(1)));

for ii = 1:P_size(1)
    subplot(dim,dim,ii);
    hold on;
    plot(t_array(1:dec:end),min(2*std_p(ii,1:dec:end),1),'b');
    plot(t_array(1:dec:end),max(-2*std_p(ii,1:dec:end),-1),'b');
    plot(t_array(1:dec:end),min(max(e_array(ii,1:dec:end),-1),1));
end

figure(2),clf;
b = bar3(abs(P_full))
for k = 1:length(b)
    zdata = b(k).ZData;
    b(k).CData = zdata;
    b(k).FaceColor = 'interp';
end

