%================================================================
% 功能：   非齐次线性方程求通解中的特解 dK*dQ=dPe
% 参数：        dK，dPe, dQ理论的关节误差        
% 返回值：       CaldqT 计算的关节误差向量 
%               CaldqT  计算的关节误差矩阵  
%               Caldb   计算的{S-B}微分误差向量
% 主要思路：
% 备注：    
% 调用方法：
% 日期：    2014/9/10 
%================================================================  

%%%%%%1.%去除相关列向量后求的的一个特解dq1

% dK=[1 2 3; 2 5 7; 3 6 9; 4 10 14];
% dPe=[13;30;39;60];
% Desdq=[2; 1; 3];
dK=N'*N;
dPe=N'*M;
Desdq1=[DesdqR;Do2o1';Dte2';De1s'];
Desdq=zeros(48,1);

[RdK,jbdK]=rref(dK,100);
Z=[1:width(RdK)];
Z(jbdK)=[];
% Desdq=[DesdqR;Desdb'];
sum=zeros(length(RdK),1);
for i=1:length(Z)
    temp=dK(:,Z(i))*Desdq(Z(i));
    sum=sum+temp;
end
b=dPe(:)-sum;
dq1=dK(:,jbdK)\b;
CaldQ=Desdq;
for i=1:length(jbdK)
    CaldQ(jbdK(:,i))=dq1(i);
end
dtdq=Desdq1-CaldQ;
performance=norm(dtdq)/norm(Desdq1)












