clear;
m=1; %�ݶȸ��µ�Ƶ��
miu=1;
i=1;
j=1;
o=200;%����������n�����ɳ�ʼ����������

[ RX,RY,RZ,tx,ty,tz,RA,RAT,RC,RCT,RB,RBT,ta,tb,tc,miu2,miu3,f ] = Instantiationfunc(300);%��ʼ������n����������
RA=[RA,RA];
RAT=[RAT,RAT];
ta=[ta;ta];
RB=[RB,RB];
RBT=[RBT,RBT];
tb=[tb;tb];
RC=[RC,RC];
RCT=[RCT,RCT];
tc=[tc;tc];

noir=0.3;%��ת����RARBRC���������ת���Ƕȣ��㣩
noit=0.3;%ƽ������tatbtc��������ķ��ȣ�mm��

% [a,b]=size(RA);
% for i=1:b/3
%     AA=[RA(:,(3*i-2):3*i),ta((3*i-2):3*i);[0,0,0,1]];
%     AA=inv(AA);
%     RA(:,(3*i-2):3*i)=AA(1:3,1:3);
%     ta((3*i-2):3*i)=AA(1:3,4);
% end
% for i=1:b/3
%     BB=[RB(:,(3*i-2):3*i),tb((3*i-2):3*i);[0,0,0,1]];
%     BB=inv(BB);
%     RB(:,(3*i-2):3*i)=BB(1:3,1:3);
%     tb((3*i-2):3*i)=BB(1:3,4);
% end
% for i=1:b/3
%     CC=[RC(:,(3*i-2):3*i),tc((3*i-2):3*i);[0,0,0,1]];
%     CC=inv(CC);
%     RC(:,(3*i-2):3*i)=CC(1:3,1:3);
%     tc((3*i-2):3*i)=CC(1:3,4);
% end




[ RAN,RBN,RCN,tan,tbn,tcn ] = addnoise( noir,noit,RA,RB,RC,ta,tb,tc );%�������������
[ RXS,RYS,RZS,txs,tys,tzs ] = RXRYRZsolve(RAN,RBN,RCN,tan,tbn,tcn);%����RX��RY��RZ,tx,ty,tz�ķ�ս���Ϊ��ʼ��