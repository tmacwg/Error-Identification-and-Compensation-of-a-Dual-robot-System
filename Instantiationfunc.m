function [ RX,RY,RZ,tx,ty,tz,RA,RAT,RC,RCT,RB,RBT,ta,tb,tc,miu2,miu3,f ] = Instantiationfunc(n)
%n是生成初始样本的组数
thetaX=1.3*pi;
thetaY=0.6*pi;
thetaZ=0.8*pi;
RX=[cos(thetaX),sin(thetaX),0;-sin(thetaX),cos(thetaX),0;0,0,1];
RY=[cos(thetaY),sin(thetaY),0;-sin(thetaY),cos(thetaY),0;0,0,1];
RZ=[cos(thetaZ),sin(thetaZ),0;-sin(thetaZ),cos(thetaZ),0;0,0,1];
tx=[0,0,197]';
ty=[2010,0,0]';
tz=[0,0,102]';
RA=[];
RB=[];
RC=[];
RAT=[]; 
RBT=[];
RCT=[];
ta=[];
tb=[];
tc=[];
for i=1:n
RA=[RA,orth(rand(3,3))];
RAT=[RAT,RA(:,(3*i-2):3*i)'];
RC=[RC,orth(rand(3,3))];
RCT=[RCT,RC(:,(3*i-2):3*i)'];
RB=[RB,RX'*RA(:,(3*i-2):3*i)'*RY*RC(:,(3*i-2):3*i)*RZ];
RBT=[RBT,RB(:,(3*i-2):3*i)'];
ta=[ta',1000*rand(1,3)]';
tc=[tc',1000*rand(1,3)]';
tb=[tb',(RX'*RA(:,(3*i-2):3*i)'*(RY*RC(:,(3*i-2):3*i)*tz+RY*tc((3*i-2):3*i)+ty-RA(:,(3*i-2):3*i)*tx-ta((3*i-2):3*i)))']';
end

i=3;
aa=RA(:,(3*i-2):3*i)*RX*tb((3*i-2):3*i)+RA(:,(3*i-2):3*i)*tx+ta((3*i-2):3*i);
bb=RY*RC(:,(3*i-2):3*i)*tz+RY*tc((3*i-2):3*i)+ty;

m=3;
miu=0.17;
o=300;
miu2=0 ;%900/(norm(tx)^2+norm(ty)^2+norm(tz)^2);
miu3=1;% 3/(norm(tx)+norm(ty)+norm(tz));
f=0;

% X=[RX,tx;[0,0,0],1];
% Y=[RY,ty;[0,0,0],1];
% Z=[RZ,tz;[0,0,0],1];
% A=[RA(:,1:3),ta(1:3);[0,0,0],1];
% B=[RB(:,1:3),tb(1:3);[0,0,0],1];
% C=[RC(:,1:3),tc(1:3);[0,0,0],1];
% ER=RA(:,1:3)*RX*RB(:,1:3)-RY*RC(:,1:3)*RZ;
% 
% E=A*X*B-Y*C*Z;

%A=RA(:,4:6)*RX*RB(:,4:6);
%B=RY*RC(:,4:6)*RZ;
%C=kron(RZ',eye(3))*kron(RC(:,4:6)',eye(3))*RY(:);
%MXYZ=[];
%MXYZ=[RX(:);kron(RZ',eye(3))*kron(RC(:,1:3)',eye(3))*RY(:)];
    





