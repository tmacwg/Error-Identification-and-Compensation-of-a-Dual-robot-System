function [ ER,E ] = ERROR( RAN,RBN,RCN,RXS,RYS,RZS,txs,tys,tzs,tan,tbn,tcn )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
X=[RXS,txs;[0,0,0],1];
Y=[RYS,tys;[0,0,0],1];
Z=[RZS,tzs;[0,0,0],1];
A=[RAN(:,4:6),tan(4:6);[0,0,0],1];
B=[RBN(:,4:6),tbn(4:6);[0,0,0],1];
C=[RCN(:,4:6),tcn(4:6);[0,0,0],1];
ER=RAN(:,4:6)*RXS*RBN(:,4:6)-RYS*RCN(:,4:6)*RZS;
E=A*X*B-Y*C*Z;


end

