m=1; %梯度更新的频率
miu=1;
i=1;
j=1;
o=200;%迭代次数，n是生成初始样本的组数
[a,b]=size(A);
for i=1:a/4
    RAN(:,(3*i-2):3*i)=A((4*i-3):(4*i-1),1:3);
    tan((3*i-2):3*i,:)=A((4*i-3):(4*i-1),4);
    RBN(:,(3*i-2):3*i)=B((4*i-3):(4*i-1),1:3);
    tbn((3*i-2):3*i,:)=B((4*i-3):(4*i-1),4);
    RCN(:,(3*i-2):3*i)=C((4*i-3):(4*i-1),1:3);
    tcn((3*i-2):3*i,:)=C((4*i-3):(4*i-1),4);
end


[ RXS,RYS,RZS,txs,tys,tzs ] = RXRYRZsolve(RAN,RBN,RCN,tan,tbn,tcn);%计算RX、RY、RZ,tx,ty,tz的封闭解作为初始解
txs=txs/1000;
tys=tys/1000;
tzs=tzs/1000;

% ER=RAN(1:3,1:3)*RXS*RBN(1:3,1:3)-RYS*RCN(1:3,1:3)*RZS;