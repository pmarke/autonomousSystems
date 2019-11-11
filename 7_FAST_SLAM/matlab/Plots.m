clear all
close all
load('FastSlam_particles_200_landmarks_5.mat')


log_t= fopen("/home/mark/projects/autonomousSystems/7_FAST_SLAM/log/t.log","r");
log_x= fopen("/home/mark/projects/autonomousSystems/7_FAST_SLAM/log/x.log","r");
log_xh= fopen("/home/mark/projects/autonomousSystems/7_FAST_SLAM/log/xh.log","r");
% log_u= fopen("/home/mark/projects/autonomousSystems/7_FAST_SLAM/log/u.log","r");
log_P= fopen("/home/mark/projects/autonomousSystems/7_FAST_SLAM/log/P.log","r");
% log_P_full = fopen("/home/mark/projects/autonomousSystems/7_FAST_SLAM/log/P_full.log","r");
log_m= fopen("/home/mark/projects/autonomousSystems/7_FAST_SLAM/log/m.log","r");
log_landmarks= fopen("/home/mark/projects/autonomousSystems/7_FAST_SLAM/log/landmarks.log","r");
log_weights= fopen("/home/mark/projects/autonomousSystems/7_FAST_SLAM/log/weights.log","r");
log_particles= fopen("/home/mark/projects/autonomousSystems/7_FAST_SLAM/log/particles.log","r");


num_landmarks =  fread(log_landmarks,1,'int');
num_particles =  fread(log_particles,1,'int');

t_array = reshape(fread(log_t,'float'),1,[]);
x_array = reshape(fread(log_x,'float'),3,[]);
xh_array = reshape(fread(log_xh,'float'),3+num_landmarks*2,[]);
P_array = reshape(fread(log_P,'float32'),4*num_landmarks,num_particles,[]);
m_array = reshape(fread(log_m,'float32'),2,num_landmarks,[]);
landmarks = reshape(fread(log_landmarks,'float'),2,[]);
weights = reshape(fread(log_weights,'float'),num_particles,[]);
particles = reshape(fread(log_particles,'float'),3+num_landmarks*2,num_particles,[]);



P_best = zeros(4*num_landmarks,length(t_array));
particle_best = zeros(3+2*num_landmarks,length(t_array));
std_p = zeros(2*num_landmarks,length(t_array));

for ii = 1:length(t_array)
    [Y,I] = max(weights(:,ii));
    P_best(:,ii) = reshape(P_array(:,I,ii),4*num_landmarks,[]);
    particle_best(:,ii) = reshape(particles(:,I,ii),3+num_landmarks*2,[]);
    for jj = 1:num_landmarks
       std_p(2*jj-1,ii) = real(sqrt(P_best(4*jj-3,ii)));
       std_p(2*jj,ii) = real(sqrt(P_best(4*jj,ii)));
    end
end

% u_array = reshape(fread(log_u,'float'),2,[]);

% P_full = reshape(fread(log_P_full,'float32'),3+num_landmarks*2,3+num_landmarks*2);

% std_p = real(sqrt(P_best));


% std_p = sqrt(P_array);
% 
% 
tmp = repmat(reshape(landmarks,1,[])',1,length(t_array));
e_array = [x_array;tmp] - xh_array;
dec = 10;

% fclose(log_K);
fclose(log_t);
fclose(log_x);
fclose(log_xh);
% fclose(log_u);
fclose(log_P);
fclose(log_m);
fclose(log_landmarks);
fclose(log_weights);
fclose(log_particles);


ra = RobotAnimation(landmarks,x_array);


%%
for ii = 2:length(t_array)
% %     ii
   ra.Update(particle_best(:,ii),x_array(:,ii),std_p(:,ii),reshape(m_array(1,:,ii),1,[]),reshape(m_array(1,:,ii),1,[]));

   pause(0.01)
end
ra.drawEstimateTrack(xh_array(1:3,:))



%%

figure(1),clf
P_size = size(P_array);
dim = ceil(sqrt(P_size(1)/2));

for ii = 1:num_landmarks*2
    subplot(dim,dim,ii);
    hold on;
    plot(t_array(1:dec:end),min(2*std_p(ii,1:dec:end),1),'b');
    plot(t_array(1:dec:end),max(-2*std_p(ii,1:dec:end),-1),'b');
    plot(t_array(1:dec:end),min(max(e_array(ii+3,1:dec:end),-1),1));
end

figure(2),clf;
for k = 1:3
    subplot(3,1,k);
    hold on;
    plot(t_array(1:dec:end),min(max(e_array(k,1:dec:end),-1),1));
end

%%
save('FastSlam_particles_1000_landmarks_10_fov_45.mat','t_array','x_array','xh_array','P_array','m_array','landmarks','weights','particles','num_landmarks','num_particles');



