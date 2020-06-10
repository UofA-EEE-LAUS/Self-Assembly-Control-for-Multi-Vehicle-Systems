function J = obstaclefunction(x,w1,obstacles) 
% �ϰ�������Ƴ�����
% �˹��Ƴ�������ˮ�»�����·���滮�����������Χ

% ���ߣ�����
% ��λ���Ϻ����´�ѧˮ�»�����������ϵͳʵ����
% Date: 2008-10-30
% Modified: 2010-1-5, 2014-11-12, 2018-3-5
% Shanghai, China 

sigma = 1.5; % ������
%r = 2.25; % �ϰ����ۺϰ뾶Ϊ1.5

[m,n] = size(obstacles);
dist = x*ones(1,n)-obstacles; % 2*1��1*n������ˣ��൱�ڵ�ֵ��չ��
dist = sum(dist.^2); % ����ƽ�����
J = min(dist); 
%if J >= 0.2
     J = w1*5.5*exp(-sigma*J); %��ָ����Ϊ�Ƴ�����
%else 
%    J = w1*4.5*exp(-sigma*0.2);
%end