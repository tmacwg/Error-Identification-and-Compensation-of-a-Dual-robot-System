m=1; %�ݶȸ��µ�Ƶ��
miu=1;
i=1;
j=1;
o=200;%����������n�����ɳ�ʼ����������
[a,b]=size(A);
for i=1:a/4
    RAN(:,(3*i-2):3*i)=A((4*i-3):(4*i-1),1:3);
    tan((3*i-2):3*i,:)=A((4*i-3):(4*i-1),4);
    RBN(:,(3*i-2):3*i)=B((4*i-3):(4*i-1),1:3);
    tbn((3*i-2):3*i,:)=B((4*i-3):(4*i-1),4);
    RCN(:,(3*i-2):3*i)=C((4*i-3):(4*i-1),1:3);
    tcn((3*i-2):3*i,:)=C((4*i-3):(4*i-1),4);
end


[ RXS,RYS,RZS,txs,tys,tzs ] = RXRYRZsolve(RAN,RBN,RCN,tan,tbn,tcn);%����RX��RY��RZ,tx,ty,tz�ķ�ս���Ϊ��ʼ��
txs=txs/1000;
tys=tys/1000;
tzs=tzs/1000;

% ER=RAN(1:3,1:3)*RXS*RBN(1:3,1:3)-RYS*RCN(1:3,1:3)*RZS;