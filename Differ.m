%构建Ti的全微分表达式，并推导出△a，△alpha，△d，△theta，△beta

syms Tibeta invTibeta a ialpha d itheta ibeta Da Dialpha Dd Ditheta Dibeta

Tibeta=[                          cos(ibeta)*cos(itheta),                          -cos(ibeta)*sin(itheta),          sin(ibeta),      a + d*sin(ibeta);
cos(ialpha)*sin(itheta) + cos(itheta)*sin(ialpha)*sin(ibeta), cos(ialpha)*cos(itheta) - sin(ialpha)*sin(ibeta)*sin(itheta), -cos(ibeta)*sin(ialpha), -cos(ibeta)*d*sin(ialpha);
sin(ialpha)*sin(itheta) - cos(ialpha)*cos(itheta)*sin(ibeta), cos(itheta)*sin(ialpha) + cos(ialpha)*sin(ibeta)*sin(itheta),  cos(ialpha)*cos(ibeta),  cos(ialpha)*cos(ibeta)*d;
                                       0,                                        0,               0,                 1];

invTibeta=inv([                          cos(ibeta)*cos(itheta),                          -cos(ibeta)*sin(itheta),          sin(ibeta),      a + d*sin(ibeta);
cos(ialpha)*sin(itheta) + cos(itheta)*sin(ialpha)*sin(ibeta), cos(ialpha)*cos(itheta) - sin(ialpha)*sin(ibeta)*sin(itheta), -cos(ibeta)*sin(ialpha), -cos(ibeta)*d*sin(ialpha);
sin(ialpha)*sin(itheta) - cos(ialpha)*cos(itheta)*sin(ibeta), cos(itheta)*sin(ialpha) + cos(ialpha)*sin(ibeta)*sin(itheta),  cos(ialpha)*cos(ibeta),  cos(ialpha)*cos(ibeta)*d;
                                       0,                                        0,               0,                 1]);

DTibeta=diff(Tibeta, 'a')*Da+diff(Tibeta, 'ialpha')*Dialpha+diff(Tibeta, 'd')*Dd+diff(Tibeta, 'itheta')*Ditheta+diff(Tibeta, 'ibeta')*Dibeta;
I=[1 0 0 0;0 1 0 0;0 0 1 0; 0 0 0 1];
DD=invTibeta*DTibeta; % 论文中的k△表达式

Da=0;
DD_Da=DD-subs(DD); 
syms Da
DD_Da=DD_Da/Da; %计算△a

syms Da Dialpha Dd Ditheta Dibeta
Dialpha=0;
DD_Dialpha=DD-subs(DD); 
syms Dialpha
DD_Dialpha=DD_Dialpha/Dialpha;  %计算△alpha

syms Da Dialpha Dd Ditheta Dibeta
Dd=0;
DD_Dd=DD-subs(DD); 
syms Dd
DD_Dd=DD_Dd/Dd; %计算△d

syms Da Dialpha Dd Ditheta Dibeta
Ditheta=0;
DD_Ditheta=DD-subs(DD);
syms Ditheta
DD_Ditheta=DD_Ditheta/Ditheta;%计算△theta

syms Da Dialpha Dd Ditheta Dibeta
Dibeta=0;
DD_Dibeta=DD-subs(DD); 
syms Dibeta
DD_Dibeta=DD_Dibeta*inv(Dibeta);%计算△beta


a=15; ialpha=141; d=141; itheta=114; ibeta=131; Da=111; Dialpha=1; Dd=1111; Ditheta=1111; Dibeta=1111;
aaa=DD-(DD_Dibeta*Dibeta+DD_Ditheta*Ditheta+DD_Dd*Dd+DD_Dialpha*Dialpha+DD_Da*Da);%检验论文的式38是否成立
aaaa=double(subs(aaa));  


%% 测试误差 
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

% DesQ=[Q(:,1:2),Theta',Q(:,4:5)]+dQ;
% QQ=[Q(:,1:2),Theta',Q(:,4:5)];
% 
% t1_1=CalculateT1(DesQ(1,:));%机器人第一个轴的位姿变换矩阵的设计值
% TDt1=t1_1-t1;  % 机器人第一个轴的位姿变换矩阵的设计值减去测量值
% Da=dQ(1,2);
% Dialpha=dQ(1,1);
% Dd=dQ(1,4);
% Ditheta=dQ(1,3);
% Dibeta=dQ(1,5);
% a=QQ(1,2);
% ialpha=QQ(1,1); 
% d=QQ(1,4);
% itheta=QQ(1,3);
% ibeta=QQ(1,5);
% error_j1=TDt1-t1*double(subs(DD));
% 
% t2_2=CalculateT1(DesQ(2,:));%机器人第二个轴的位姿变换矩阵的设计值
% TDt2=t2_2-t2;  % 机器人第二个轴的位姿变换矩阵的设计值减去测量值
% Da=dQ(2,2);
% Dialpha=dQ(2,1);
% Dd=dQ(2,4);
% Ditheta=dQ(2,3);
% Dibeta=dQ(2,5);
% a=QQ(2,2);
% ialpha=QQ(2,1); 
% d=QQ(2,4);
% itheta=QQ(2,3);
% ibeta=QQ(2,5);
% error_j2=TDt2-t2*double(subs(DD));
% 
% t3_3=CalculateT1(DesQ(3,:));%机器人第三个轴的位姿变换矩阵的设计值
% TDt3=t3_3-t3;  % 机器人第三个轴的位姿变换矩阵的设计值减去测量值
% Da=dQ(3,2);
% Dialpha=dQ(3,1);
% Dd=dQ(3,4);
% Ditheta=dQ(3,3);
% Dibeta=dQ(3,5);
% a=QQ(3,2);
% ialpha=QQ(3,1); 
% d=QQ(3,4);
% itheta=QQ(3,3);
% ibeta=QQ(3,5);
% error_j3=TDt3-t3*double(subs(DD));
% 
% t4_4=CalculateT1(DesQ(4,:));%机器人第四个轴的位姿变换矩阵的设计值
% TDt4=t4_4-t4;  % 机器人第四个轴的位姿变换矩阵的设计值减去测量值
% Da=dQ(4,2);
% Dialpha=dQ(4,1);
% Dd=dQ(4,4);
% Ditheta=dQ(4,3);
% Dibeta=dQ(4,5);
% a=QQ(4,2);
% ialpha=QQ(4,1); 
% d=QQ(4,4);
% itheta=QQ(4,3);
% ibeta=QQ(4,5);
% error_j4=TDt4-t4*double(subs(DD));
% 
% t5_5=CalculateT1(DesQ(5,:));%机器人第五个轴的位姿变换矩阵的设计值
% TDt5=t5_5-t5;  % 机器人第五个轴的位姿变换矩阵的设计值减去测量值
% Da=dQ(5,2);
% Dialpha=dQ(5,1);
% Dd=dQ(5,4);
% Ditheta=dQ(5,3);
% Dibeta=dQ(5,5);
% a=QQ(5,2);
% ialpha=QQ(5,1); 
% d=QQ(5,4);
% itheta=QQ(5,3);
% ibeta=QQ(5,5);
% error_j5=TDt5-t5*double(subs(DD));
% 
% t6_6=CalculateT1(DesQ(6,:));%机器人第二个轴的位姿变换矩阵的设计值
% TDt6=t6_6-t6;  % 机器人第二个轴的位姿变换矩阵的设计值减去测量值
% Da=dQ(6,2);
% Dialpha=dQ(6,1);
% Dd=dQ(6,4);
% Ditheta=dQ(6,3);
% Dibeta=dQ(6,5);
% a=QQ(6,2);
% ialpha=QQ(6,1); 
% d=QQ(6,4);
% itheta=QQ(6,3);
% ibeta=QQ(6,5);
% error_j6=TDt6-t6*double(subs(DD));

