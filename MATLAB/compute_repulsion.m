%��������
function [Yrerxx,Yreryy]=compute_repulsion(X,Xsum,m,angle_re,n,Po)%�������Ϊ��ǰ���꣬Xsum��Ŀ����ϰ����������������泣��,�ϰ���Ŀ�귽��ĽǶ�
  for i=1:n
    Rrei(i)=(X(1)-Xsum(i+1,1))^2+(X(2)-Xsum(i+1,2))^2;%·������ϰ��ľ���ƽ��
    rre(i)=sqrt(Rrei(i));%·������ϰ��ľ��뱣��������rrei��
    if rre(i)>Po%���ÿ���ϰ���·���ľ�������ϰ�Ӱ����룬������Ϊ0
          Yrerx(i)=0
          Yrery(i)=0
    else
       Yrer(i)=m*(1/rre(i)-1/Po)^2*1/(rre(i)^2)%�ֽ��Fre1����
       Yrerx(i)=Yrer(i)*cos(angle_re(i))%angle_re(i)=Y(i+1)
       Yrery(i)=Yrer(i)*sin(angle_re(i))
    end%�жϾ����Ƿ����ϰ�Ӱ�췶Χ��
end
   Yrerxx=sum(Yrerx)%���ӳ����ķ���
   Yreryy=sum(Yrery)