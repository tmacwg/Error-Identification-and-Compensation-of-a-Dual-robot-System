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
[RdK,jbdK]=rref(dK);
%%找出1对应的行数
njbdK=size(jbdK,2);
RjbdK=zeros(njbdK,1);
for i=1:size(jbdK,2)
    for j=1:size(dK,1)
         if RdK(j,jbdK(:,i))==1
             RjbdK(i,1)=j;%%%找出线性无关向量中元素"1"对应的行数，默认为1->njbdK
             break;
         end
        
    end
end
dq1=dK(:,jbdK)\dPe;


%%%%%%2.计算不去除相关列向量后的一个特解 CaldQ
ndq1=size(dq1,1);
dq2=zeros(ndq1,1);
Desdq=[DesdqR;Desdb'];
for i=1:ndq1
    dq2(i,:)=dq1(i,:)-RdK(RjbdK(i,:),:)*Desdq+Desdq(jbdK(:,i),1);%先求解 CaldQ中包含dq1变量的值
end
CaldQ=Desdq;
for i=1:njbdK
    CaldQ(jbdK(:,i),:)=dq2(i,:);%%求解 CaldQ中包含dq1变量的值为dq2,，不包含的变量值等于理论值Desdq中相应的变量
end

%%%%%%3.求解对应的关节参数误差矩阵CaldqT和{S-B}微分误差矢量Caldb
DesdqT=dQ;
CaldqT=[CaldQ(7:12,:),CaldQ(1:6,:),CaldQ(19:24,:),CaldQ(13:18,:),CaldQ(25:30,:)];
Desdb=Desdq(31:36,:);
Caldb=CaldQ(31:36,:);


%%%%%%%4.验证
dtdq=CaldQ-Desdq;
dtdb=Caldb-Desdb;
dtdQT=CaldqT-DesdqT;













%s=5;
    
