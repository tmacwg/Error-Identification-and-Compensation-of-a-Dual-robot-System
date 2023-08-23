function G=CalG_Dualrobot(iQ,To2s1,Te1t1,Ttt,MeaTte2,Theta)
%================================================================
% 功能：   四元数Q转旋转矩阵RR  版本2 版本1为CalculateK
% 参数：   
%         iQ=[q0 q1 q2 q3];
% 返回值：  RR为3X3的旋转矩阵
% 主要思路：
% 备注：    
% 调用方法：
% 日期：    2014/8/23 20:37
%================================================================
%%%%%%%%%%求解在一个姿态Ti下的K
QQ=iQ;
%真实使用时，上式中的theta是需要用机器人运动学来求解的。 

% 每个关节每个微分参数的系数矩阵
[A_Dbx A_Dby A_Dbz A_Dbrx A_Dbry A_Dbrz]=CalculateA0(QQ);
[A_Alpha1 A_a1 A_Theta1  A_d1 A_beta1]=CalculateAi_wg(QQ(1,:));
[A_Alpha2 A_a2 A_Theta2  A_d2 A_beta2]=CalculateAi_wg(QQ(2,:));
[A_Alpha3 A_a3 A_Theta3  A_d3 A_beta3]=CalculateAi_wg(QQ(3,:));
[A_Alpha4 A_a4 A_Theta4  A_d4 A_beta4]=CalculateAi_wg(QQ(4,:));
[A_Alpha5 A_a5 A_Theta5  A_d5 A_beta5]=CalculateAi_wg(QQ(5,:));
[A_Alpha6 A_a6 A_Theta6  A_d6 A_beta6]=CalculateAi_wg(QQ(6,:));


% 计算相邻关节的位姿矩阵
t1=CalculateT1(QQ(1,:));%T01
t2=CalculateT1(QQ(2,:));%T12
t3=CalculateT1(QQ(3,:));%T23
t4=CalculateT1(QQ(4,:));%T34
t5=CalculateT1(QQ(5,:));%T45
t6=CalculateT1(QQ(6,:));%T56

%% ============================测试==============================
dQ=[0.0054    0.0036    0.0015    0.0024         0
    0.0010    0.0010    0.0033    0.0023    0.0023
    0.0010    0.0006    0.0010    0.0023    0.0041
    0.0019    0.0018    0.0010    0.0010         0
    0.0051    0.0061    0.0023    0.0010         0
    0.0023    0.0007    0.0010    0.0020         0];

%alpha         a     theta         d      beta
Q =  [0         0         0    0.6300         0
    -1.5708    0.6000   -1.5708         0         0
    0    1.2800         0         0         0
    -1.5708    0.2000    3.1416    1.5920         0
    -1.5708         0         0         0         0
    1.5708         0         0    0.2000         0];

DesQ=[Q(:,1:2),Theta',Q(:,4:5)]+dQ;
t1_1=CalculateT1(DesQ(1,:));%机器人第一个轴的位姿变换矩阵的设计值
TDt1=t1_1-t1;  % 机器人第一个轴的位姿变换矩阵的设计值减去测量值
error_j1=TDt1-t1*(A_Alpha1*dQ(1,1)+A_a1*dQ(1,2)+A_Theta1*dQ(1,3)+A_d1*dQ(1,4)+A_beta1*dQ(1,5));

t2_2=CalculateT1(DesQ(2,:));%机器人第二个轴的位姿变换矩阵的设计值
TDt2=t2_2-t2;  % 机器人第二个轴的位姿变换矩阵的设计值减去测量值
error_j2=TDt2-t2*(A_Alpha2*dQ(2,1)+A_a2*dQ(2,2)+A_Theta2*dQ(2,3)+A_d2*dQ(2,4)+A_beta2*dQ(2,5));

t3_3=CalculateT1(DesQ(3,:));%机器人第三个轴的位姿变换矩阵的设计值
TDt3=t3_3-t3;  % 机器人第三个轴的位姿变换矩阵的设计值减去测量值
error_j3=TDt3-t3*(A_Alpha3*dQ(3,1)+A_a3*dQ(3,2)+A_Theta3*dQ(3,3)+A_d3*dQ(3,4)+A_beta3*dQ(3,5));

t4_4=CalculateT1(DesQ(4,:));%机器人第四个轴的位姿变换矩阵的设计值
TDt4=t4_4-t4;  % 机器人第四个轴的位姿变换矩阵的设计值减去测量值
error_j4=TDt4-t4*(A_Alpha4*dQ(4,1)+A_a4*dQ(4,2)+A_Theta4*dQ(4,3)+A_d4*dQ(4,4)+A_beta4*dQ(4,5));

t5_5=CalculateT1(DesQ(5,:));%机器人第五个轴的位姿变换矩阵的设计值
TDt5=t5_5-t5;  % 机器人第五个轴的位姿变换矩阵的设计值减去测量值
error_j5=TDt5-t5*(A_Alpha5*dQ(5,1)+A_a5*dQ(5,2)+A_Theta5*dQ(5,3)+A_d5*dQ(5,4)+A_beta5*dQ(5,5));

t6_6=CalculateT1(DesQ(6,:));%机器人第二个轴的位姿变换矩阵的设计值
TDt6=t6_6-t6;  % 机器人第二个轴的位姿变换矩阵的设计值减去测量值
error_j6=TDt6-t6*(A_Alpha6*dQ(6,1)+A_a6*dQ(6,2)+A_Theta6*dQ(6,3)+A_d6*dQ(6,4)+A_beta6*dQ(6,5));


%% ================================================================

Ts_6=To2s1*t1*t2*t3*t4*t5*t6;
T6_s=inv(Ts_6);
Tse2=MeaTte2*inv(Ttt);
T0_6=t1*t2*t3*t4*t5*t6;

TL=To2s1;TR=t1*t2*t3*t4*t5*t6*T6_s;
ADbx=TL*A_Dbx*TR;     DDbx=T2DiffVect(ADbx);
ADby=TL*A_Dby*TR;     DDby=T2DiffVect(ADby);
ADbz=TL*A_Dbz*TR;     DDbz=T2DiffVect(ADbz);
ADbrx=TL*A_Dbrx*TR;   DDbrx=T2DiffVect(ADbrx);
ADbry=TL*A_Dbry*TR;   DDbry=T2DiffVect(ADbry);
ADbrz=TL*A_Dbrz*TR;   DDbrz=T2DiffVect(ADbrz);

TL=To2s1*t1;TR=t2*t3*t4*t5*t6*T6_s;
AAlpha1=TL*A_Alpha1*TR;  DAlpha1=T2DiffVect(AAlpha1);
Aa1=TL*A_a1*TR;          Da1=T2DiffVect(Aa1);
ATheta1=TL*A_Theta1*TR;  DTheta1=T2DiffVect(ATheta1);
Ad1=TL*A_d1*TR;          Dd1=T2DiffVect(Ad1);
Abeta1=TL*A_beta1*TR;    Dbeta1=T2DiffVect(Abeta1);


TL=To2s1*t1*t2;TR=t3*t4*t5*t6*T6_s;
AAlpha2=TL*A_Alpha2*TR; DAlpha2=T2DiffVect(AAlpha2);
Aa2=TL*A_a2*TR;         Da2=T2DiffVect(Aa2);
ATheta2=TL*A_Theta2*TR; DTheta2=T2DiffVect(ATheta2);
Ad2=TL*A_d2*TR;          Dd2=T2DiffVect(Ad2);
Abeta2=TL*A_beta2*TR;    Dbeta2=T2DiffVect(Abeta2);


TL=To2s1*t1*t2*t3;TR=t4*t5*t6*T6_s;
AAlpha3=TL*A_Alpha3*TR;  DAlpha3=T2DiffVect(AAlpha3);
Aa3=TL*A_a3*TR;           Da3=T2DiffVect(Aa3);
ATheta3=TL*A_Theta3*TR;   DTheta3=T2DiffVect(ATheta3);
Ad3=TL*A_d3*TR;           Dd3=T2DiffVect(Ad3);
Abeta3=TL*A_beta3*TR;      Dbeta3=T2DiffVect(Abeta3);

TL=To2s1*t1*t2*t3*t4;TR=t5*t6*T6_s; 
AAlpha4=TL*A_Alpha4*TR; DAlpha4=T2DiffVect(AAlpha4);
Aa4=TL*A_a4*TR;         Da4=T2DiffVect(Aa4);
ATheta4=TL*A_Theta4*TR;  DTheta4=T2DiffVect(ATheta4);
Ad4=TL*A_d4*TR;           Dd4=T2DiffVect(Ad4);
Abeta4=TL*A_beta4*TR;     Dbeta4=T2DiffVect(Abeta4);

TL=To2s1*t1*t2*t3*t4*t5;TR=t6*T6_s; 
AAlpha5=TL*A_Alpha5*TR; DAlpha5=T2DiffVect(AAlpha5);
Aa5=TL*A_a5*TR;         Da5=T2DiffVect(Aa5);
ATheta5=TL*A_Theta5*TR;   DTheta5=T2DiffVect(ATheta5);
Ad5=TL*A_d5*TR;           Dd5=T2DiffVect(Ad5);
Abeta5=TL*A_beta5*TR;     Dbeta5=T2DiffVect(Abeta5);

TL=To2s1*t1*t2*t3*t4*t5*t6;TR=T6_s; 
AAlpha6=TL*A_Alpha6*TR; DAlpha6=T2DiffVect(AAlpha6);
Aa6=TL*A_a6*TR;          Da6=T2DiffVect(Aa6);
ATheta6=TL*A_Theta6*TR;   DTheta6=T2DiffVect(ATheta6);
Ad6=TL*A_d6*TR;            Dd6=T2DiffVect(Ad6);
Abeta6=TL*A_beta6*TR;     Dbeta6=T2DiffVect(Abeta6);

Da=[Da1 Da2 Da3 Da4 Da5 Da6];
DAlpha=[DAlpha1 DAlpha2 DAlpha3 DAlpha4 DAlpha5 DAlpha6];
Dd=[Dd1 Dd2 Dd3 Dd4 Dd5 Dd6];
DTheta=[DTheta1 DTheta2 DTheta3 DTheta4 DTheta5 DTheta6];
Dbeta=[Dbeta1 Dbeta2 Dbeta3 Dbeta4 Dbeta5 Dbeta6];
DDb=[DDbx DDby DDbz DDbrx DDbry DDbrz];
%%1-6  7-12 13-18 19-24 25-30 31-36

DDo2s=Adv(To2s1);
DDe1s=Adv(Te1t1);
DDts=Adv(Ttt);

G=[Da DAlpha Dd  DTheta Dbeta DDo2s DDe1s DDts];

