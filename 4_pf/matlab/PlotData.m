clear all
close all

log_chi= fopen("/home/mark/tmp/AS_particle/chi.log","r");
log_t= fopen("/home/mark/tmp/AS_particle/t.log","r");
log_x= fopen("/home/mark/tmp/AS_particle/x.log","r");
log_xh= fopen("/home/mark/tmp/AS_particle/xh.log","r");
log_u= fopen("/home/mark/tmp/AS_particle/u.log","r");
% log_P= fopen("/home/mark/tmp/AS_particle/P.log","r");
log_m= fopen("/home/mark/tmp/AS_particle/m.log","r");




chi_array = reshape(fread(log_chi,'float'),4,1000,[]);
t_array = reshape(fread(log_t,'float'),1,[]);
x_array = reshape(fread(log_x,'float'),3,[]);
xh_array = reshape(fread(log_xh,'float'),3,[]);
u_array = reshape(fread(log_u,'float'),2,[]);
% P_array = reshape(fread(log_P,'float'),3,3,[]);
num_meas = fread(log_m,1,'int');
m_pos = reshape(fread(log_m,2*num_meas,'float'),2,num_meas,[]);
% m_array = reshape(fread(log_m,'float32'),2,num_meas,[]);
% m_array = fread(log_m,'float32');


[xx,yy,zz] = size(chi_array);
P_array = zeros(3,3,zz);
for ii = 1:zz
    for jj = 1:yy
    P_array(:,:,ii) = P_array(:,:,ii) +chi_array(4,jj,ii).*(chi_array(1:3,jj,ii)-xh_array(:,ii)).*(chi_array(1:3,jj,ii)-xh_array(:,ii))';

    end
end


std_px = reshape(P_array(1,1,:),1,[]);
std_px = sqrt(std_px);
std_py = reshape(P_array(2,2,:),1,[]);
std_py = sqrt(std_py);
std_w = reshape(P_array(3,3,:),1,[]);
std_w = sqrt(std_w);

e_array = x_array - xh_array;
dec = 10;

fclose(log_chi);
fclose(log_t);
fclose(log_x);
fclose(log_xh);
fclose(log_u);
% fclose(log_P);
% fclose(log_m);

%%
offset = 0.1;
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
ylim([-5*std_px(end)-offset,5*std_px(end)+offset])

subplot(3,1,2)
plot(t_array,2*std_py,'b')
hold on
plot(t_array,-2*std_py,'b')
plot(t_array,e_array(2,:),'r')
legend("2*std_dev","-2*std_dev","error")
title("Position Y")
xlabel("Seconds")
ylabel("Meters")
ylim([-5*std_py(end)-offset,5*std_py(end)+offset])

subplot(3,1,3)
plot(t_array,2*std_w,'b')
hold on
plot(t_array,-2*std_w,'b')
plot(t_array,e_array(3,:),'r')
legend("2*std dev","-2*std dev","error")
title("Angular Velocity")
xlabel("Seconds")
ylabel("Radians/Sec")
ylim([-5*std_w(end)-offset,5*std_w(end)+offset])

% [row,col] = size(K_array(:,:,1));
% figure(3),clf;
% title("Kalman Gains")
% for r = 1:row
%    for c = 1:col
%         subplot(row,col,r*col+c-col)
%         plot(t_array,reshape(K_array(r,c,:),1,[]))
%    end
%     
% end

%%
animation_dec = 10;
for i = 1:animation_dec:length(x_array)
    
    animate(t_array(i),x_array(:,i),xh_array(:,i),num_meas,m_pos,reshape(chi_array(:,:,i),4,[]))
    pause(0.5)
    
end
