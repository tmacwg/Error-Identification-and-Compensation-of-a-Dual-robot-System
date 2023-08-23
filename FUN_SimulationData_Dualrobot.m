function [matAng,Desbt,Destb,DesTse1,DesTe1s,DesTo1o2,DesTo2o1,DesTe1t,DesTte2,DesTse,MeaTse,DesTo1e1,simPs] = FUN_SimulationData_Dualrobot(numPt,Q,dQ,DesQ,DesdqR)
%%%%%%1 设定位姿数目和对应的关节角
% numPt=400;%位姿数目9
%在仿真中，客观世界中实际的值是Des值，观测的或出厂设定并读取的值是Mea值

matAng=(rand(numPt,6)-0.5)*2*180/180*pi;%产生numPt行 6列分布在0~1的随机数矩阵
qi_Base=[0 -90 0 180 0 0]/180*pi;
 for i=1:numPt
      matAng(i,:)=qi_Base+matAng(i,:);%关节角度矩阵
 end
 

% %%%%%%  2 设定关节连杆参数矩阵Q、和理论的关节误差矩阵dQ
%     %alpha         a     theta         d      beta
%    Q =  [0         0         0    0.6300         0
%    -1.5708    0.6000   -1.5708         0         0
%          0    1.2800         0         0         0
%    -1.5708    0.2000    3.1416    1.5920         0
%    -1.5708         0         0         0         0
%     1.5708         0         0    0.2000         0];
    
% dQ=[0.0054    0.0036    0.0015    0.0024         0
%     0.0010    0.0010    0.0033    0.0023    0.0023
%     0.0010    0.0006    0.0010    0.0023    0.0041
%     0.0019    0.0018    0.0010    0.0010         0
%     0.0051    0.0061    0.0023    0.0010         0
%     0.0023    0.0007    0.0010    0.0020         0];

% DesQ=Q+dQ;
% DesdqR=[dQ(:,2);dQ(:,1);dQ(:,4);dQ(:,3);dQ(:,5)];%转为误差向量


%%%%%%  3  设定numPt/2 组{T}下的球心坐标DesPe
Rbt=eye(3,3);
DesPe=[0, 0, 0]/1000;
Desbt=[Rbt,DesPe';[0 0 0 1]];
Destb=inv(Desbt);
% DesPe=(rand(numPt,3)-0.5)*2*1000/1000;%%平移向量
% DesPe(numPt/2+1:numPt,:)=DesPe(1:numPt/2,:);

%%%%%%  4  设定{S-E1}位姿的理论值DesTe1s
Rse1=rot([0 1 0]',90);
tse1=[0 0 300/1000]';
DesTse1=[Rse1,tse1;[0 0 0 1]];
DesTe1s=inv(DesTse1);

%%%%%%  5  设定{O1-O2}位姿的理论值DesTo2o1
Ro1o2=rot([0 0 1]',180);
to1o2=[0 0 2000/1000]';
DesTo1o2=[Ro1o2,to1o2;[0 0 0 1]];
DesTo2o1=inv(DesTo1o2);

%%%%%%  6  设定{E2-T}位姿的理论值DesTte2
Re1t=rot([0 1 0]',90);
te1t=[150/1000 0 200/1000]';
DesTe1t=[Re1t,te1t;[0 0 0 1]];
DesTte2=inv(DesTe1t);


%%%%%%  7  模拟出numPt个加工机器人的理论位姿和实际位姿
MeaTse=zeros(4,4,numPt);
DesTse=zeros(4,4,numPt);
for i=1:numPt
tmpiQ=[DesQ(:,1:2),matAng(i,:)'+dQ(:,3),DesQ(:,4:5)];
DesTse(:,:,i)=MyFkine(tmpiQ);  %理论加工机器人位姿
MeaTse(:,:,i)=MyFkine([Q(:,1:2),matAng(i,:)',Q(:,4:5)]);  %实际读取的含误差的加工机器人位姿MeaTe2o2
end

%%%%%%  8  模拟出numPt个测量系统的理论位姿和实际位姿
RA=[];
DesTo1e1=[];
for i=1:numPt
    RA=orth(rand(3,3));
    ta=1000*rand(1,3)'/1000;
    TempDesTo1e1=[RA,ta;[0 0 0 1]];
    DesTo1e1(:,:,i)=TempDesTo1e1; %认为测量系统误差为0，生成的数据无误差
end




%%%%%%%  9  模拟出扫描仪下面的球心坐标simPs
simPs=zeros(numPt,4);
Desbs=zeros(numPt*4,4);
for i=1:numPt
simPs(i,:)=((DesTe1s*DesTo1e1(:,:,i)*DesTo2o1*DesTse(:,:,i))*DesTte2*([DesPe 1]'))';  %不含误差，即认为扫描仪测量值是准的
end
simPs=simPs(:,1:3); 
end