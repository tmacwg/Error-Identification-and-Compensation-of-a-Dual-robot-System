function [ C] = Ccalculate( C0 )
%UNTITLED6 Summary of this function goes here
%   Detailed explanation goes here
[a,b]=size(C0);
for i=1:a
  R((3*i-2):3*i,:)=RPY2R_KUKA(C0(i,4:6));
 % R((3*i-2):3*i,:)=OulerToRota(C0(i,4:6));
  t(i,:)=C0(i,1:3);
  C((4*i-3):4*i,:)=[R((3*i-2):3*i,:),t(i,:)';[0,0,0,1]];
  %C((4*i-3):4*i,:)=[R((3*i-2):3*i,:),[0,0,0]';[0,0,0,1]]*[eye(3), t(i,:)';[0,0,0,1]];
 %C((4*i-3):4*i,:)=inv(C((4*i-3):4*i,:));
%    t1=inv(C((4*i-3):4*i,:))*[t(i,:),1]';
end

end

