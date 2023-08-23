function [ A ] = Acalculate( A0 )
%UNTITLED7 Summary of this function goes here
%   Detailed explanation goes here
[a,b]=size(A0);
for i=1:a
    R=Q2R(A0(i,4:7));
    t=A0(i,1:3)';
    A((4*i-3):4*i,:)=[R,t;[0,0,0,1]];
 %   A((4*i-3):4*i,:)=inv(A((4*i-3):4*i,:));
%t1=inv(A((4*i-3):4*i,:))*[t;1];
end
end