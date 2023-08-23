function [CaldQ,CaldQ1] = FUN_Solution_dq_wg_experiment(N,M)
%================================================================
% 功能：   非齐次线性方程求通解中的特解 dK*dQ=dPe
% 参数：        dK，dPe, dQ理论的关节误差        
% 返回值：       CaldqT 计算的关节误差向量 
%               CaldqT  计算的关节误差矩阵  
%               Caldb   计算的{S-B}微分误差向量
% 主要思路：
% 备注：    
% 调用方法：
% 日期：    2022/10/25 
%================================================================  

%%%%%%1.%去除相关列向量后求的的一个特解dq1

% dK=[1 2 3; 2 5 7; 3 6 9; 4 10 14];
% dPe=[13;30;39;60];
% Desdq=[2; 1; 3];
dK=N'*N;
dPe=N'*M;
% Desdq1=[DesdqR;Do2o1';De1s';Dte2'];
Desdq=zeros(48,1);

[RdK,jbdK]=rref(dK,1);
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
% dtdq=Desdq1(jbdK)-CaldQ(jbdK);

% performance=norm(dtdq)/norm(Desdq1)  %越小越好，越小表示计算值与设计值dq之间的偏差越小，越接近

CaldQ1=tls(N,M);

% B=norm(Desdq1-CaldQ)/norm(Desdq1)
% A=norm(Desdq1-CaldQ1)/norm(Desdq1)
end