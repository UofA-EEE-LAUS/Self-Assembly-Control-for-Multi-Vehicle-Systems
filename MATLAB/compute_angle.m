function Y=compute_angle(X,Xsum,n)%Y��������������x��ĽǶ�����,X��������꣬Xsum��Ŀ����ϰ�����������,��(n+1)*2����
for i=1:n+1%n���ϰ���Ŀ
deltaXi=Xsum(i,1)-X(1)
deltaYi=Xsum(i,2)-X(2)
ri=sqrt(deltaXi^2+deltaYi^2)
if deltaXi>0
theta=asin(deltaXi/ri)
else
theta=pi-asin(deltaXi/ri)
end
if i==1%��ʾ��Ŀ��
angle=theta
else
angle=pi+theta
end 
Y(i)=angle%����ÿ���Ƕ���Y�������棬��һ��Ԫ������Ŀ��ĽǶȣ����涼�����ϰ��ĽǶ�
end
