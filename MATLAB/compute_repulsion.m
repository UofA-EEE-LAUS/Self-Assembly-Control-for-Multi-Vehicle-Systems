%斥力计算
function [Yrerxx,Yreryy]=compute_repulsion(X,Xsum,m,angle_re,n,Po)%输入参数为当前坐标，Xsum是目标和障碍的坐标向量，增益常数,障碍，目标方向的角度
  for i=1:n
    Rrei(i)=(X(1)-Xsum(i+1,1))^2+(X(2)-Xsum(i+1,2))^2;%路径点和障碍的距离平方
    rre(i)=sqrt(Rrei(i));%路径点和障碍的距离保存在数组rrei中
    if rre(i)>Po%如果每个障碍和路径的距离大于障碍影响距离，斥力令为0
          Yrerx(i)=0
          Yrery(i)=0
    else
       Yrer(i)=m*(1/rre(i)-1/Po)^2*1/(rre(i)^2)%分解的Fre1向量
       Yrerx(i)=Yrer(i)*cos(angle_re(i))%angle_re(i)=Y(i+1)
       Yrery(i)=Yrer(i)*sin(angle_re(i))
    end%判断距离是否在障碍影响范围内
end
   Yrerxx=sum(Yrerx)%叠加斥力的分量
   Yreryy=sum(Yrery)