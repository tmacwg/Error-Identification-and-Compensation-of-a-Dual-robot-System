%================================================================
% 功能：   机器人标定
% 参数：   Ps1、Ps2 在两个姿态下所测量的n组特征点在扫描仪下的坐标[x y z]'；维数3Xn
%          E1、E2   n组特征点的姿态 e=[x y z q0 q1 q2 q3]' 维数7Xn
%          Q        六个关节的关节参数   [alpha a theta d beta]
%          matAng1、matAng2 n组位姿下对应的关节角
% 返回值：  dq为各关节的补偿值
% 主要思路：先求解初值Tbs，再补偿
% 备注：    
% 调用方法：
% 日期：    2014/8/23 20:37
%================================================================
Ps1=simPs(1:numPt/2,:)';  Ps2=simPs(numPt/2+1:numPt,:)'; %3*n
E1=MeaTse(:,:,1:numPt/2);   E2=MeaTse(:,:,numPt/2+1:numPt);%7*n
% E1=DesTse(:,:,1:numPt/2);   E2=DesTse(:,:,numPt/2+1:numPt);%7*n
matAng1=matAng(1:numPt/2,:);matAng2=matAng(numPt/2+1:numPt,:);
M1=DesTo1e1(:,:,1:numPt/2);  M2=DesTo1e1(:,:,numPt/2+1:numPt);
%%%%%%%%%%%%%%%%%%%%%%%%
  
% %%%%%%%%步骤1：求解机器人基坐标系与扫描仪之间的位姿Tbs%%%%%%%%%%%%%
% %b表示基坐标系 s表示扫描仪坐标系
% 
% nPt=size(Ps1,2);%所测点数
% 
% %步骤1.1 求解dM、dPbe
% dM=zeros(3*nPt,12);%初始化dM大小
% dPbe=zeros(3*nPt,1);%初始化dPbe大小
% for i=1:nPt
%     T1i=inv(E1(:,:,i));
%     T2i=inv(E2(:,:,i));%计算在测量第i个特征点时，机器人末端相对基坐标系的位姿二
%     M1i=[kron(Ps1(:,i)',T1i(1:3,1:3)),T1i(1:3,1:3)];%计算第一个姿态下的M矩阵
%     M2i=[kron(Ps2(:,i)',T2i(1:3,1:3)),T2i(1:3,1:3)];%计算第二个姿态下的M矩阵
%    
%     P1bei=T1i(1:3,4);%计算机器人末端相对于基坐标系的姿态
%     P2bei=T2i(1:3,4);
%    
%     dM(3*i-2:3*i,:)=M1i-M2i;
%     dPbe(3*i-2:3*i,1)=P2bei-P1bei;
% end
% 
% %步骤1.2 最小二乘求解Tbs
% X=inv(dM'*dM)*dM'*dPbe;
% invTbs=[X(1:3,1),X(4:6,1),X(7:9,1),X(10:12,1);0 0 0 1];
% MyTbs=invTbs;
% tmpTbs=inv(invTbs);
% tmpTbs=T2StdT(tmpTbs);%转为标准正交矩阵
% 
% %%%%%%%%%%%%% START                         计算badTsb
% % dTbs=[0.1,0.2,0.3,0.4,0.5,0.6]/1000;
% % badtbs=tbs-[1 2 4]*0.1/1000;
% % badRpy=Rpy-[2 5 6]*0.1/1000;
% % badTsb=[RPY2R(badRpy),badtbs';0 0 0 1];%用人为设定的Tbs
% %badTsb=Tbs; %用真实的Tbs 
% %%%%%%%%%%%%%  END
% 
% %%%%%%%%%%% START                         求解理论的 dTbs  
% dT=tmpTbs\(DesTbs-tmpTbs);
% %%%%求有标准baddT的矩阵Tsb
% dT2=D2T(T2D(dT));
% MeaTbs=DesTbs/((eye(4)+dT2));
% Desdb=T2D(dT2)';
% %%%%%%%%%%%  END


%%%%%%%%步骤1：人工给定测量末端标系与扫描仪之间的位姿Te1s的误差%%%%%%%%%%%%%
nPt=size(Ps1,2);%所测点数
k=2*rand(3,1)-[1,1,1]';
k=k/norm(k);
R=DesTe1s(1:3,1:3)*rot(k,0.01);
k=k*0.0001;
t=DesTe1s(1:3,4)+k;
MeaTe1s=[R,t;[0 0 0 1]];

dT=MeaTe1s\(DesTe1s-MeaTe1s);
dT2=D2T(T2D(dT));
De1s=T2D(dT)';
%%%%%%%%%%%  END

%%%%%%%%步骤2：人工给定刀具坐标系与加工机器人末端之间的位姿Tte2的误差%%%%%%%%%%%%%
nPt=size(Ps1,2);%所测点数
k=2*rand(3,1)-[1,1,1]';
k=k/norm(k);
R=DesTte2(1:3,1:3)*rot(k,0.01);
k=k*0.0001;
t=DesTte2(1:3,4)+k;
MeaTte2=[R,t;[0 0 0 1]];

dT=MeaTte2\(DesTte2-MeaTte2);
dT2=D2T(T2D(dT));
Dte2=T2D(dT)';
%%%%%%%%%%%  END


%%%%%%%%步骤3：人工给定加工机器人基坐标系与测量基坐标系之间的位姿To2o1的误差%%%%%%%%%%%%%
nPt=size(Ps1,2);%所测点数
k=2*rand(3,1)-[1,1,1]';
k=k/norm(k);
R=DesTo2o1(1:3,1:3)*rot(k,0.01);
k=k*0.0001;
t=DesTo2o1(1:3,4)+k;
MeaTo2o1=[R,t;[0 0 0 1]];

dT=MeaTo2o1\(DesTo2o1-MeaTo2o1);
dT2=D2T(T2D(dT));
Do2o1=T2D(dT)';
%%%%%%%%%%%  END







%%%%%%%%%%%%%%%%%步骤2：机器人标定 求解dq%%%%%%%%%%%%%%%%%%%%%
%步骤2.1 机器人建模
% clear IRB4400
% IRB4400=RobotModel2(Q);
%步骤2.2 求解dK、dPe
num_dq=48;%补偿参数个数
dK=zeros(3*nPt,num_dq);%初始化dK
dPe=zeros(3*nPt,1);%初始化dPe
for i=1:nPt
    %计算Ki1  Ki2  Ki1-Ki2
    %计算dP
    T1i=inv(MeaTe1s*M1(:,:,i)*MeaTo2o1*E1(:,:,i)*MeaTte2);
    T2i=inv(MeaTe1s*M2(:,:,i)*MeaTo2o1*E2(:,:,i)*MeaTte2);%计算在测量第i个特征点时，机器人末端相对扫描仪坐标系的位姿二

    To2s1=MeaTe1s*M1(:,:,i)*MeaTo2o1;
    To2s2=MeaTe1s*M2(:,:,i)*MeaTo2o1;
    Te1s=MeaTe1s;
    Tts1=MeaTe1s*M1(:,:,i)*MeaTo2o1*E1(:,:,i)*MeaTte2;
    Tts2=MeaTe1s*M2(:,:,i)*MeaTo2o1*E2(:,:,i)*MeaTte2;
    
    
%     MeaTts=(DesTe1s*DesTo1e1(:,:,i)*DesTo2o1*DesTse(:,:,i))*DesTte2;
%     MeaTst=MeaTts;
    
    Theta1i=matAng1(i,:); %逆运动学求解第一个姿态的关节变量
    Theta2i=matAng2(i,:);%逆运动学求解第二个姿态的关节变量
    Q1i=[Q(:,1:2),Theta1i',Q(:,4:5)];%更新关节参数
    Q2i=[Q(:,1:2),Theta2i',Q(:,4:5)];%更新关键参数
    K1i=CalK_Dualrobot(Q1i,To2s1,Te1s,Tts1,Ps1(:,i));  %计算K1
    K2i=CalK_Dualrobot(Q2i,To2s2,Te1s,Tts2,Ps2(:,i));  %计算K2
    Pe1i=T1i(1:3,1:3)*Ps1(:,i)+T1i(1:3,4);%计算第i个特征点在机器人末端坐标系e下的坐标1
    Pe2i=T2i(1:3,1:3)*Ps2(:,i)+T2i(1:3,4);%计算第i个特征点在机器人末端坐标系e下的坐标2
    dK(3*i-2:3*i,:)=K1i-K2i; %计算dK
    dPe(3*i-2:3*i,1)=Pe2i-Pe1i; %计算在两个姿态下，特征点机器人末端坐标系e下的坐标差dP 
end
dK(:,18)=0;
s=rank(dK);
m=0;
%步骤2.3 最小二乘求解关节Q的补偿向量dq
%%%%%%%%%%%%%%  START
 %解法1 
 %rag=rank(AG); rdG=rank(dG);raK=rank(AK);rdK=rank(dK);判断是否为满秩矩阵
 %dq=inv(dK'*dK)*dK'*dPe;
 %解法2 %见Solution)_dQ.m
%%%%%%%%%%%%%%  END 
