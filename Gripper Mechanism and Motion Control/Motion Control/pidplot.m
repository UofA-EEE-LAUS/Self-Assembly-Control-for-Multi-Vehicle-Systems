load acceleration2.mat
H1=H;
t=0;
speed_pre = 0;
% load pid_2_5_0.5.mat
% H2=H;
% load pid_5_5_0.1.mat
% H3=H;
total_t = 0;
t_final=91;
figure(3);
plot(H1(1:t_final,4),-H1(1:t_final,2));
title('position');
% hold on;
% plot(H2(:,4),-H2(:,2));
% hold on;
% plot(H3(:,4),-H3(:,2));
% legend('2 5 0.1','2 5 0.5')

G=zeros([length(H), 2]);
F=zeros([length(H), 2]);
for i=2:length(H)
    j1=-H(i-1,2);
    j2=-H(i,2);
    d=j2-j1;
    t1=H(i-1,4);
    t2=H(i,4);
    t_diff=t2-t1;
    speed=(d)/t_diff;
    adspeed = (speed-speed_pre)/t_diff;
    speed_pre = speed;
    t=t+t_diff;
    G(i,:)=[speed,t];
    F(i,:)=[adspeed,t];
end
% 
% for i=2:399
%     j1=G(i-1,1);
%     j2=G(i,1);
%     d=j2-j1;
%     t1(i)=G(i,2);
%     t2(i)=G(i+1,2);
%     tdiff(i)=t2(i)-t1(i);
%     adspeed(i)=d(i)/tdiff(i);
%     t(i)=t1(i)+(tdiff(i)/2);
%     F(i,:)=[adspeed(i);t(i)];
% 
% end

figure(4);
plot(G(1:t_final,2),G(1:t_final,1))
title('speed');
figure(2);
plot(F(1:t_final,2),F(1:t_final,1))
title('acceleration');
