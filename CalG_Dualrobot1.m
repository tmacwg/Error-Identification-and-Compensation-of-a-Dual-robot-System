function [G,DDS]=CalG_Dualrobot1(iQ,To2s1,Te1t1,Ttt,MeaTte2,Theta,Do2o1,De1s,Dte2,dQ,Q)
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
% [A_Alpha1 A_a1 A_Theta1  A_d1 A_beta1]=CalculateAi_wg(QQ(1,:));
% [A_Alpha2 A_a2 A_Theta2  A_d2 A_beta2]=CalculateAi_wg(QQ(2,:));
% [A_Alpha3 A_a3 A_Theta3  A_d3 A_beta3]=CalculateAi_wg(QQ(3,:));
% [A_Alpha4 A_a4 A_Theta4  A_d4 A_beta4]=CalculateAi_wg(QQ(4,:));
% [A_Alpha5 A_a5 A_Theta5  A_d5 A_beta5]=CalculateAi_wg(QQ(5,:));
% [A_Alpha6 A_a6 A_Theta6  A_d6 A_beta6]=CalculateAi_wg(QQ(6,:));


% 计算相邻关节的位姿矩阵
t1=CalculateT1(QQ(1,:));%T01
t2=CalculateT1(QQ(2,:));%T12
t3=CalculateT1(QQ(3,:));%T23
t4=CalculateT1(QQ(4,:));%T34
t5=CalculateT1(QQ(5,:));%T45
t6=CalculateT1(QQ(6,:));%T56

%% ============================测试==============================
% dQ=[0.0054    0.0036    0.0015    0.0024         0
%     0.0010    0.0010    0.0033    0.0023    0.0023
%     0.0010    0.0006    0.0010    0.0023    0.0041
%     0.0019    0.0018    0.0010    0.0010         0
%     0.0051    0.0061    0.0023    0.0010         0
%     0.0023    0.0007    0.0010    0.0020         0];
% 
% %alpha         a     theta         d      beta
% Q =  [0         0         0    0.6300         0
%     -1.5708    0.6000   -1.5708         0         0
%     0    1.2800         0         0         0
%     -1.5708    0.2000    3.1416    1.5920         0
%     -1.5708         0         0         0         0
%     1.5708         0         0    0.2000         0];

Differ;%构建Ti的全微分表达式，并推导出△a，△alpha，△d，△theta，△beta

Da=10^2;% 初始化任意值
Dialpha=10^3;% 初始化任意值
Dd=10^4;% 初始化任意值
Ditheta=10^5;% 初始化任意值
Dibeta=10^2;% 初始化任意值

a=QQ(1,2); ialpha=QQ(1,1); d=QQ(1,4); itheta=QQ(1,3); ibeta=QQ(1,5);Dibeta=10^1;
A_a1=double(subs(DD_Da));    A_Alpha1=double(subs(DD_Dialpha));    A_d1=double(subs(DD_Dd));    A_Theta1=double(subs(DD_Ditheta));    A_beta1=double(subs(DD_Dibeta));
a=QQ(2,2); ialpha=QQ(2,1); d=QQ(2,4); itheta=QQ(2,3); ibeta=QQ(2,5);
A_a2=double(subs(DD_Da));    A_Alpha2=double(subs(DD_Dialpha));    A_d2=double(subs(DD_Dd));    A_Theta2=double(subs(DD_Ditheta));    A_beta2=double(subs(DD_Dibeta));
a=QQ(3,2); ialpha=QQ(3,1); d=QQ(3,4); itheta=QQ(3,3); ibeta=QQ(3,5);
A_a3=double(subs(DD_Da));    A_Alpha3=double(subs(DD_Dialpha));    A_d3=double(subs(DD_Dd));    A_Theta3=double(subs(DD_Ditheta));    A_beta3=double(subs(DD_Dibeta));
a=QQ(4,2); ialpha=QQ(4,1); d=QQ(4,4); itheta=QQ(4,3); ibeta=QQ(4,5);
A_a4=double(subs(DD_Da));    A_Alpha4=double(subs(DD_Dialpha));    A_d4=double(subs(DD_Dd));    A_Theta4=double(subs(DD_Ditheta));    A_beta4=double(subs(DD_Dibeta));
a=QQ(5,2); ialpha=QQ(5,1); d=QQ(5,4); itheta=QQ(5,3); ibeta=QQ(5,5);
A_a5=double(subs(DD_Da));    A_Alpha5=double(subs(DD_Dialpha));    A_d5=double(subs(DD_Dd));    A_Theta5=double(subs(DD_Ditheta));    A_beta5=double(subs(DD_Dibeta));
a=QQ(6,2); ialpha=QQ(6,1); d=QQ(6,4); itheta=QQ(6,3); ibeta=QQ(6,5);
A_a6=double(subs(DD_Da));    A_Alpha6=double(subs(DD_Dialpha));    A_d6=double(subs(DD_Dd));    A_Theta6=double(subs(DD_Ditheta));    A_beta6=double(subs(DD_Dibeta));



%% ================================================================

Ts_6=To2s1*t1*t2*t3*t4*t5*t6;
T6_s=inv(Ts_6);
Tse2=MeaTte2*inv(Ttt);
T0_6=t1*t2*t3*t4*t5*t6;

TL=To2s1;TR=t1*t2*t3*t4*t5*t6*T6_s;
ADbx=TL*A_Dbx*TR;     DDbx=T2D(ADbx);
ADby=TL*A_Dby*TR;     DDby=T2D(ADby);
ADbz=TL*A_Dbz*TR;     DDbz=T2D(ADbz);
ADbrx=TL*A_Dbrx*TR;   DDbrx=T2D(ADbrx);
ADbry=TL*A_Dbry*TR;   DDbry=T2D(ADbry);
ADbrz=TL*A_Dbrz*TR;   DDbrz=T2D(ADbrz);

TL=To2s1*t1;TR=t2*t3*t4*t5*t6*T6_s;
AAlpha1=TL*A_Alpha1*TR;  DAlpha1=T2D(AAlpha1);
Aa1=TL*A_a1*TR;          Da1=T2D(Aa1);
ATheta1=TL*A_Theta1*TR;  DTheta1=T2D(ATheta1);
Ad1=TL*A_d1*TR;          Dd1=T2D(Ad1);
Abeta1=TL*A_beta1*TR;    Dbeta1=T2D(Abeta1);

Da=dQ(1,2);
Dialpha=dQ(1,1);
Dd=dQ(1,4);
Ditheta=dQ(1,3);
Dibeta=dQ(1,5);
a=QQ(1,2);
ialpha=QQ(1,1); 
d=QQ(1,4);
itheta=QQ(1,3);
ibeta=QQ(1,5);
DD1=TL*double(subs(DD))*TR; 


TL=To2s1*t1*t2;TR=t3*t4*t5*t6*T6_s;
AAlpha2=TL*A_Alpha2*TR; DAlpha2=T2D(AAlpha2);
Aa2=TL*A_a2*TR;         Da2=T2D(Aa2);
ATheta2=TL*A_Theta2*TR; DTheta2=T2D(ATheta2);
Ad2=TL*A_d2*TR;          Dd2=T2D(Ad2);
Abeta2=TL*A_beta2*TR;    Dbeta2=T2D(Abeta2);

Da=dQ(2,2);
Dialpha=dQ(2,1);
Dd=dQ(2,4);
Ditheta=dQ(2,3);
Dibeta=dQ(2,5);
a=QQ(2,2);
ialpha=QQ(2,1); 
d=QQ(2,4);
itheta=QQ(2,3);
ibeta=QQ(2,5);
DD2=TL*double(subs(DD))*TR;

TL=To2s1*t1*t2*t3;TR=t4*t5*t6*T6_s;
AAlpha3=TL*A_Alpha3*TR;  DAlpha3=T2D(AAlpha3);
Aa3=TL*A_a3*TR;           Da3=T2D(Aa3);
ATheta3=TL*A_Theta3*TR;   DTheta3=T2D(ATheta3);
Ad3=TL*A_d3*TR;           Dd3=T2D(Ad3);
Abeta3=TL*A_beta3*TR;      Dbeta3=T2D(Abeta3);

Da=dQ(3,2);
Dialpha=dQ(3,1);
Dd=dQ(3,4);
Ditheta=dQ(3,3);
Dibeta=dQ(3,5);
a=QQ(3,2);
ialpha=QQ(3,1); 
d=QQ(3,4);
itheta=QQ(3,3);
ibeta=QQ(3,5);
DD3=TL*double(subs(DD))*TR;

TL=To2s1*t1*t2*t3*t4;TR=t5*t6*T6_s; 
AAlpha4=TL*A_Alpha4*TR; DAlpha4=T2D(AAlpha4);
Aa4=TL*A_a4*TR;         Da4=T2D(Aa4);
ATheta4=TL*A_Theta4*TR;  DTheta4=T2D(ATheta4);
Ad4=TL*A_d4*TR;           Dd4=T2D(Ad4);
Abeta4=TL*A_beta4*TR;     Dbeta4=T2D(Abeta4);

Da=dQ(4,2);
Dialpha=dQ(4,1);
Dd=dQ(4,4);
Ditheta=dQ(4,3);
Dibeta=dQ(4,5);
a=QQ(4,2);
ialpha=QQ(4,1); 
d=QQ(4,4);
itheta=QQ(4,3);
ibeta=QQ(4,5);
DD4=TL*double(subs(DD))*TR;

TL=To2s1*t1*t2*t3*t4*t5;TR=t6*T6_s; 
AAlpha5=TL*A_Alpha5*TR; DAlpha5=T2D(AAlpha5);
Aa5=TL*A_a5*TR;         Da5=T2D(Aa5);
ATheta5=TL*A_Theta5*TR;   DTheta5=T2D(ATheta5);
Ad5=TL*A_d5*TR;           Dd5=T2D(Ad5);
Abeta5=TL*A_beta5*TR;     Dbeta5=T2D(Abeta5);

Da=dQ(5,2);
Dialpha=dQ(5,1);
Dd=dQ(5,4);
Ditheta=dQ(5,3);
Dibeta=dQ(5,5);
a=QQ(5,2);
ialpha=QQ(5,1); 
d=QQ(5,4);
itheta=QQ(5,3);
ibeta=QQ(5,5);
DD5=TL*double(subs(DD))*TR;

TL=To2s1*t1*t2*t3*t4*t5*t6;TR=T6_s; 
AAlpha6=TL*A_Alpha6*TR; DAlpha6=T2D(AAlpha6);
Aa6=TL*A_a6*TR;          Da6=T2D(Aa6);
ATheta6=TL*A_Theta6*TR;   DTheta6=T2D(ATheta6);
Ad6=TL*A_d6*TR;            Dd6=T2D(Ad6);
Abeta6=TL*A_beta6*TR;     Dbeta6=T2D(Abeta6);

Da=dQ(6,2);
Dialpha=dQ(6,1);
Dd=dQ(6,4);
Ditheta=dQ(6,3);
Dibeta=dQ(6,5);
a=QQ(6,2);
ialpha=QQ(6,1); 
d=QQ(6,4);
itheta=QQ(6,3);
ibeta=QQ(6,5);
DD6=TL*double(subs(DD))*TR;

DDS=To2s1*D2T(Do2o1)*inv(To2s1)+Te1t1*D2T(De1s)*inv(Te1t1)+Ttt*D2T(Dte2)*inv(Ttt)+DD1+DD2+DD3+DD4+DD5+DD6; %对应论文公式44的右端


Daal=[Da1 Da2 Da3 Da4 Da5 Da6];
DAlpha=[DAlpha1 DAlpha2 DAlpha3 DAlpha4 DAlpha5 DAlpha6];
Dd=[Dd1 Dd2 Dd3 Dd4 Dd5 Dd6];
DTheta=[DTheta1 DTheta2 DTheta3 DTheta4 DTheta5 DTheta6];
Dbeta=[Dbeta1 Dbeta2 Dbeta3 Dbeta4 Dbeta5 Dbeta6];
DDb=[DDbx DDby DDbz DDbrx DDbry DDbrz];
%%1-6  7-12 13-18 19-24 25-30 31-36

DDo2s=Adv(To2s1);
DDe1s=Adv(Te1t1);
DDts=Adv(Ttt);

G=[Daal DAlpha Dd  DTheta Dbeta DDo2s DDe1s DDts];

DesdqR=[dQ(:,2);dQ(:,1);dQ(:,4);dQ(:,3);dQ(:,5)];
Desdq1=[DesdqR;Do2o1;De1s;Dte2];

sumQ1=Daal*DesdqR(1:6)+DAlpha*DesdqR(7:12)+Dd*DesdqR(13:18)+DTheta*DesdqR(19:24)+Dbeta*DesdqR(25:30); %小论文式44的第一行中与Q相关的部分
sumQ2=DD1+DD2+DD3+DD4+DD5+DD6;  %小论文式44的第二行中与Q相关的部分，为了检验sumQ2和sumQ1是否相等



