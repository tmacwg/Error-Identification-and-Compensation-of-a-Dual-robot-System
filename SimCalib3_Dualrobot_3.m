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
E1_Des=DesTse(:,:,1:numPt/2);   E2_Des=DesTse(:,:,numPt/2+1:numPt);%7*n
matAng1=matAng(1:numPt/2,:);matAng2=matAng(numPt/2+1:numPt,:);
M1=DesTo1e1(:,:,1:numPt/2);  M2=DesTo1e1(:,:,numPt/2+1:numPt);


RCN=[];
RBN=[];
RAN=[];
tan=[];
tbn=[];
tcn=[];
for i=1:numPt
    if i<=numPt/2
        RCN=[RCN,E1(1:3,1:3,i)];  %含噪音的加工机器人姿态
        tcn=[tcn;E1(1:3,4,i)];
        MM1=inv(M1(:,:,i));
        RAN=[RAN,MM1(1:3,1:3)];   %不含噪音的测量机器人姿态
        tan=[tan;MM1(1:3,4)];
        S1=DesTe1s*M1(:,:,i)*DesTo2o1*E1_Des(:,:,i)*DesTte2;
        RBN=[RBN,S1(1:3,1:3)];    %工具相对扫描仪的姿态，不含噪音
        tbn=[tbn;S1(1:3,4)]; 
    else
        RCN=[RCN,E2(1:3,1:3,i-numPt/2)];
        tcn=[tcn;E2(1:3,4,i-numPt/2)];
        MM2=inv(M2(:,:,i-numPt/2));
        RAN=[RAN,MM2(1:3,1:3)];
        tan=[tan;MM2(1:3,4)];
        S2=DesTe1s*M2(:,:,i-numPt/2)*DesTo2o1*E2_Des(:,:,i-numPt/2)*DesTte2;
        RBN=[RBN,S2(1:3,1:3)];
        tbn=[tbn;S2(1:3,4)];
    end
end

[ RXS,RYS,RZS,txs,tys,tzs ] = RXRYRZsolve(RAN,RBN,RCN,tan,tbn,tcn);%计算RX、RY、RZ,tx,ty,tz的封闭解作为初始解
ER=RAN(1:3,1:3)*RXS*RBN(1:3,1:3)-RYS*RCN(1:3,1:3)*RZS;



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


%%%%%%%%步骤1：人工给定测量末端标系与扫描仪之间的位姿Te1s的误差%%%%%%%%%%%%%  X
% k=2*rand(3,1)-[1,1,1]';
% k=k/norm(k);
% % R=DesTe1s(1:3,1:3)*rot(k,0.5);
% k1=k*0.0001;
% % t=DesTe1s(1:3,4)+k1;
% % MeaTe1s=[R,t;[0 0 0 1]]; %含误差的矩阵Te1s
% % dT=MeaTe1s\(DesTe1s-MeaTe1s); 
% % dT2=D2T(T2D(dT));
% R_deta=rot(k,0.7);
% R_deta(1,1)=0;
% R_deta(2,2)=0;
% R_deta(3,3)=0;
% R_deta=(R_deta-R_deta')/2;
% dT=[R_deta,k1;[0 0 0 0]];


X=[RXS,txs;[0 0 0],1];
X=inv(X);
dT=inv(X)*DesTe1s;
dT(1,1)=0;
dT(2,2)=0;
dT(3,3)=0;
dT(4,4)=0;
% dT=dT/1000;
De1s=T2D(dT)';
MeaTe1s=DesTe1s/((eye(4)+dT));
%%%%%%%%%%%  END

%%%%%%%%步骤2：人工给定刀具坐标系与加工机器人末端之间的位姿Tte2的误差%%%%%%%%%%%%%  Z
% k=2*rand(3,1)-[1,1,1]';
% k=k/norm(k);
% % R=DesTte2(1:3,1:3)*rot(k,0.5);
% k1=k*0.0001;
% % t=DesTte2(1:3,4)+k1;
% % MeaTte2=[R,t;[0 0 0 1]]; %含误差的矩阵Tte2
% % dT=MeaTte2\(DesTte2-MeaTte2);
% % dT2=D2T(T2D(dT));
% R_deta=rot(k,0.3);
% R_deta(1,1)=0;
% R_deta(2,2)=0;
% R_deta(3,3)=0;
% R_deta=(R_deta-R_deta')/2;
% dT=[R_deta,k1;[0 0 0 0]];

Z=[RZS,tzs;[0 0 0],1];
dT=inv(Z)*DesTte2;
dT(1,1)=0;
dT(2,2)=0;
dT(3,3)=0;
dT(4,4)=0;
% dT=dT/1000;
Dte2=T2D(dT)';
MeaTte2=DesTte2/((eye(4)+dT));
%%%%%%%%%%%  END


%%%%%%%%步骤3：人工给定加工机器人基坐标系与测量基坐标系之间的位姿To2o1的误差%%%%%%%%%%%%%  Y
% nPt=size(Ps1,2);%所测点数
% k=2*rand(3,1)-[1,1,1]';
% k=k/norm(k);
% % R=DesTo2o1(1:3,1:3)*rot(k,0.5);
% k1=k*0.0001;
% % t=DesTo2o1(1:3,4)+k1;
% % MeaTo2o1=[R,t;[0 0 0 1]];  %含误差的矩阵To2o1
% % dT=MeaTo2o1\(DesTo2o1-MeaTo2o1);
% R_deta=rot(k,0.5);
% R_deta(1,1)=0;
% R_deta(2,2)=0;
% R_deta(3,3)=0;
% R_deta=(R_deta-R_deta')/2;
% dT=[R_deta,k1;[0 0 0 0]];

Y=[RYS,tys;[0 0 0],1];
dT=inv(Y)*DesTo2o1;
dT(1,1)=0;
dT(2,2)=0;
dT(3,3)=0;
dT(4,4)=0;
% dT=dT/1000;
Do2o1=T2D(dT)';
MeaTo2o1=DesTo2o1/((eye(4)+dT));
%%%%%%%%%%%  END


%%%%%%%%%%% 计算含误差的位姿变换矩阵Tst
for i=1:numPt/2
    MeaTts1(i*4-3:i*4,:)=MeaTe1s*M1(:,:,i)*MeaTo2o1*E1(:,:,i)*MeaTte2;
    MeaTst1(i*4-3:i*4,:)=inv(MeaTts1(i*4-3:i*4,:)); %第一组数据的含误差的位姿变换矩阵Tst
    MeaTts2(i*4-3:i*4,:)=MeaTe1s*M2(:,:,i)*MeaTo2o1*E2(:,:,i)*MeaTte2;
    MeaTst2(i*4-3:i*4,:)=inv(MeaTts2(i*4-3:i*4,:)); %第二组数据的含误差的位姿变换矩阵Tst
end
%%%%%%%%%%%  END


%%%%%%%%%%% 计算含误差的位姿变换矩阵Tbt
% Desbs(i*4-3:i*4,:)=DesTe1s*DesTo1e1(:,:,i)*DesTo2o1*DesTse(:,:,i)*DesTte2*Desbt;  %不含误差B到s的变换矩阵Tbs
for i=1:numPt/2
    Desbs1(i*4-3:i*4,:)=DesTe1s*M1(:,:,i)*DesTo2o1*E1_Des(:,:,i)*DesTte2*Desbt; %不含误差真实的B到s的变换矩阵Tbs
    Desbs2(i*4-3:i*4,:)=DesTe1s*M2(:,:,i)*DesTo2o1*E2_Des(:,:,i)*DesTte2*Desbt;
    MeaTbt1(i*4-3:i*4,:)=MeaTst1(i*4-3:i*4,:)*Desbs1(i*4-3:i*4,:); %含误差的位姿变换矩阵Tbt
    MeaTbt2(i*4-3:i*4,:)=MeaTst2(i*4-3:i*4,:)*Desbs2(i*4-3:i*4,:);
end
%%%%%%%%%%%  END
nPt=size(Ps1,2);%所测点数

%%%%%%%%%%%%%%%%%步骤2：计算矩阵G%%%%%%%%%%%%%%%%%%%%%
G1=[];
G2=[];
for i=1:nPt
    To2s1=MeaTe1s*M1(:,:,i)*MeaTo2o1;
    To2s2=MeaTe1s*M2(:,:,i)*MeaTo2o1;
    Tso21=inv(To2s1);
    Tso22=inv(To2s2);
        
    Tts1=MeaTe1s*M1(:,:,i)*MeaTo2o1*E1(:,:,i)*MeaTte2; 
    Tts2=MeaTe1s*M2(:,:,i)*MeaTo2o1*E2(:,:,i)*MeaTte2; 
     
    Theta1i=matAng1(i,:); %逆运动学求解第一个姿态的关节变量
    Theta2i=matAng2(i,:); %逆运动学求解第二个姿态的关节变量
    Q1i=[Q(:,1:2),Theta1i',Q(:,4:5)];%更新关节参数
    Q2i=[Q(:,1:2),Theta2i',Q(:,4:5)];%更新关节参数
    G1i=CalG_Dualrobot(Q1i,To2s1,Tts1,MeaTe1s,MeaTte2);  %计算G1
    G2i=CalG_Dualrobot(Q2i,To2s2,Tts2,MeaTe1s,MeaTte2);  %计算G2
    G1=[G1;G1i];
    G2=[G2;G2i];
end
%%%%%%%%%%%  END
Desdq=[DesdqR;Do2o1';Dte2';De1s'];

%%%%%%%%%%%%%%%%%计算矩阵M和N %%%%%%%%%%%%%%%%%%%%%
for i=1:nPt

    KRx2(3*i-2:3*i,:)=MeaTbt2(i*4-3:i*4-1,1:3)*[0 0 0;0 0 -1;0 1 0]*Desbs2(i*4-3:i*4-1,1:3)'*G2(6*i-2:6*i,:);
    KRx1(3*i-2:3*i,:)=MeaTbt1(i*4-3:i*4-1,1:3)*[0 0 0;0 0 -1;0 1 0]*Desbs1(i*4-3:i*4-1,1:3)'*G1(6*i-2:6*i,:);
    KRy2(3*i-2:3*i,:)=MeaTbt2(i*4-3:i*4-1,1:3)*[0 0 1;0 0 0;-1 0 0]*Desbs2(i*4-3:i*4-1,1:3)'*G2(6*i-2:6*i,:);
    KRy1(3*i-2:3*i,:)=MeaTbt1(i*4-3:i*4-1,1:3)*[0 0 1;0 0 0;-1 0 0]*Desbs1(i*4-3:i*4-1,1:3)'*G1(6*i-2:6*i,:);
    KRz2(3*i-2:3*i,:)=MeaTbt2(i*4-3:i*4-1,1:3)*[0 -1 0;1 0 0;0 0 0]*Desbs2(i*4-3:i*4-1,1:3)'*G2(6*i-2:6*i,:);
    KRz1(3*i-2:3*i,:)=MeaTbt1(i*4-3:i*4-1,1:3)*[0 -1 0;1 0 0;0 0 0]*Desbs1(i*4-3:i*4-1,1:3)'*G1(6*i-2:6*i,:);
    K1(3*i-2:3*i,:)=MeaTts1(4*i-3:4*i-1,1:3)'*[-eye(3,3),Antisymmetric(Desbs1(i*4-3:i*4-1,4))]*G1(6*i-5:6*i,:);
    K2(3*i-2:3*i,:)=MeaTts2(4*i-3:4*i-1,1:3)'*[-eye(3,3),Antisymmetric(Desbs2(i*4-3:i*4-1,4))]*G2(6*i-5:6*i,:);
    N(12*i-11:12*i,:)=[KRx2(3*i-2:3*i,:)-KRx1(3*i-2:3*i,:);KRy2(3*i-2:3*i,:)-KRy1(3*i-2:3*i,:)...
        ;KRz2(3*i-2:3*i,:)-KRz1(3*i-2:3*i,:);K1(3*i-2:3*i,:)-K2(3*i-2:3*i,:)];
    M(12*i-11:12*i,:)=[MeaTbt1(i*4-3:i*4-1,1:3)*[1 0 0]'-MeaTbt2(i*4-3:i*4-1,1:3)*[1 0 0]';...
        MeaTbt1(i*4-3:i*4-1,1:3)*[0 1 0]'-MeaTbt2(i*4-3:i*4-1,1:3)*[0 1 0]'; ...
        MeaTbt1(i*4-3:i*4-1,1:3)*[0 0 1]'-MeaTbt2(i*4-3:i*4-1,1:3)*[0 0 1]';...
        MeaTbt2(i*4-3:i*4-1,4)-MeaTbt1(i*4-3:i*4-1,4)];
end

% deltaq=inv(N'*N)*N'*M;
Solution_dq_wg
