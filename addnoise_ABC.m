function [ AN,BN,CN ] = addnoise_ABC( noir,noit,A,B,C )
% 旋转矩阵沿随机轴线旋转小于noir°的随机角度，作为噪音
% 平移向量沿随机矢量增加小于noit mm的随机距离，作为噪音
RAN=[];
RBN=[];
RCN=[];
tan=[];
tbn=[];
tcn=[];
[a,~]=size(A);
n=a/4;
for i=1:n
    RA(:,(3*i-2):3*i)=A(i*4-3:i*4-1,1:3);
    RB(:,(3*i-2):3*i)=B(i*4-3:i*4-1,1:3);
    RC(:,(3*i-2):3*i)=C(i*4-3:i*4-1,1:3);
    ta((3*i-2):3*i)=A(i*4-3:i*4-1,4);
    tb((3*i-2):3*i)=B(i*4-3:i*4-1,4);
    tc((3*i-2):3*i)=C(i*4-3:i*4-1,4);


    kA=2*rand(3,1)-[1,1,1]';
    kAA=kA/norm(kA);
    RAN=[RAN,RA(:,(3*i-2):3*i)*rot(kAA,rand(1,1)*noir)];
    kC=2*rand(3,1)-[1,1,1]';
    kCC=kC/norm(kC);
    RCN=[RCN,RC(:,(3*i-2):3*i)*rot(kCC,rand(1,1)*noir)];
    kB=2*rand(3,1)-[1,1,1]';
    kBB=kB/norm(kB);
    RBN=[RBN,RB(:,(3*i-2):3*i)*rot(kBB,rand(1,1)*noir)];
    ka=2*rand(3,1)-[1,1,1]';
    kaa=ka/norm(ka);
    tan=[tan',(ta((3*i-2):3*i)+kaa*noit)']';
    kc=2*rand(3,1)-[1,1,1]';
    kcc=kc/norm(kc);
    tcn=[tcn',(tc((3*i-2):3*i)+kcc*noit)']';
    kb=2*rand(3,1)-[1,1,1]';
    kbb=kb/norm(kb);
    tbn=[tbn',(tb((3*i-2):3*i)+kbb*noit)']';

    AN(i*4-3:i*4-1,1:3)=RAN(:,(3*i-2):3*i);
    BN(i*4-3:i*4-1,1:3)=RBN(:,(3*i-2):3*i);
    CN(i*4-3:i*4-1,1:3)=RCN(:,(3*i-2):3*i);
    AN(i*4-3:i*4-1,4)=tan((3*i-2):3*i);
    BN(i*4-3:i*4-1,4)=tbn((3*i-2):3*i);
    CN(i*4-3:i*4-1,4)=tcn((3*i-2):3*i);



end

end

