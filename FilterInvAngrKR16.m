function [MatlabAngr StudioAngr]=FilterInvAngrKR16(iTheta)

Theta0=([0 90 0 0 0 180]/180*pi)';%%%6700的基准角度；
MatlabTheta8=iTheta;
%%将matlab的角度换算成Studio的角度
nArray=size(MatlabTheta8,2);
StudioTheta=MatlabTheta8;
for i=1:nArray
    StudioTheta(:,i)=StudioTheta(:,i)+Theta0;
%     StudioTheta(3,i)= StudioTheta(3,i)+ StudioTheta(2,i);%第三个角度为第二个角度加第三个角度
end
StudioTheta=Ang2StdAngr(StudioTheta);

%删选机器人的Studio角度范围
LastStudioTheta=[];
NumValSol=[];
% Min2=-42/180*pi; Max2=85/180*pi;Min3=-20/180*pi; Max3=120/180*pi;Min5=-120/180*pi; Max5=120/180*pi;

Min1=-185/180*pi; Max1=185/180*pi;Min2=-185/180*pi; Max2=65/180*pi;Min3=-138/180*pi; Max3=175/180*pi;Min4=-350/180*pi; Max4=350/180*pi;Min5=-130/180*pi; Max5=130/180*pi;Min6=-350/180*pi; Max6=350/180*pi;%现场考虑布线后的轴约束

for i=1:nArray
%     if StudioTheta(2,i)>Min2 && StudioTheta(2,i)<Max2 && StudioTheta(3,i)>Min3 &&StudioTheta(3,i)<Max3 && StudioTheta(5,i)>Min5 &&StudioTheta(5,i)<Max5
    if StudioTheta(1,i)>Min1 && StudioTheta(1,i)<Max1 &&StudioTheta(2,i)>Min2 && StudioTheta(2,i)<Max2 && StudioTheta(3,i)>Min3 &&StudioTheta(3,i)<Max3 &&  StudioTheta(4,i)>Min4 &&StudioTheta(4,i)<Max4  && StudioTheta(5,i)>Min5 &&StudioTheta(5,i)<Max5 &&StudioTheta(6,i)>Min6 &&StudioTheta(6,i)<Max6 
    LastStudioTheta=[LastStudioTheta StudioTheta(:,i)];
    NumValSol=[NumValSol i];
    end
end
LastMatlabTheta=[];
for i=1:size(NumValSol,2)
    LastMatlabTheta=[LastMatlabTheta MatlabTheta8(:,NumValSol(1,i))];
end
MatlabAngr=LastMatlabTheta;
StudioAngr=LastStudioTheta;