function [ xnum] = wuliao( RA,RB,RC,RX,RY,RZ,o)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here
[a,b]=size(RA);
n=b/3;
F=[];
c=[];
RX0=RX;
RY0=RY;
RZ0=RZ;
%x(:,1:12)=[RX+orth(rand(3,3))/3,RY+orth(rand(3,3))/3,RZ+orth(rand(3,3))/3,tx+2*rand(3,1),ty+2*rand(3,1),tz+2*rand(3,1)];
% 加噪音
% kX=2*rand(3,1)-[1,1,1]';
% kXX=kX/norm(kX);
% RX=RX*rot(kXX,1);
% kY=2*rand(3,1)-[1,1,1]';
% kYY=kY/norm(kY);
% RY=RY*rot(kYY,1);
% kZ=2*rand(3,1)-[1,1,1]';
% kZZ=kZ/norm(kZ);
% RZ=RZ*rot(kYY,1);


j=1;
r=0;
if o>=1
    for k=1:o
        for i=1:n
            F=[F;[-RA(:,(3*i-2):3*i)*skew(RX*RB(:,(3*i-2):3*i),1),skew(RY*RC(:,(3*i-2):3*i)*RZ,1),RY*RC(:,(3*i-2):3*i)*skew(RZ,1);...
                -RA(:,(3*i-2):3*i)*skew(RX*RB(:,(3*i-2):3*i),2),skew(RY*RC(:,(3*i-2):3*i)*RZ,2),RY*RC(:,(3*i-2):3*i)*skew(RZ,2);...
                -RA(:,(3*i-2):3*i)*skew(RX*RB(:,(3*i-2):3*i),3),skew(RY*RC(:,(3*i-2):3*i)*RZ,3),RY*RC(:,(3*i-2):3*i)*skew(RZ,3);]];
            T=-RA(:,(3*i-2):3*i)*RX*RB(:,(3*i-2):3*i)+RY*RC(:,(3*i-2):3*i)*RZ;
            c=[c;T(:,1);T(:,2);T(:,3)];
        end
        r=inv(F'*F)*F'*c;
        
%         e1=RA(:,(3*1-2):3*1)*skew(r(1:3),1)*RX*RB(:,(3*1-2):3*1)-skew(r(4:6),1)*RY*RC(:,(3*1-2):3*1)*RZ-RY*RC(:,(3*1-2):3*1)*skew(r(7:9),1)*RZ+RA(:,(3*1-2):3*1)*RX*RB(:,(3*1-2):3*1)-RY*RC(:,(3*1-2):3*1)*RZ;
%         e2=F*r-c;
        
        RX=(eye(3)+skew(r(1:3),1)+skew(r(1:3),1)^2/2+skew(r(1:3),1)^3/6+skew(r(1:3),1)^4/24)*RX;
        RY=(eye(3)+skew(r(4:6),1)+skew(r(4:6),1)^2/2+skew(r(4:6),1)^3/6+skew(r(4:6),1)^4/24)*RY;
        RZ=(eye(3)+skew(r(7:9),1)+skew(r(7:9),1)^2/2+skew(r(7:9),1)^3/6+skew(r(7:9),1)^4/24)*RZ;
        
%         d=(eye(3)+skew(r(1:3),1)+skew(r(1:3),1)^2/2+skew(r(1:3),1)^3/6+skew(r(1:3),1)^4/24);
%         e=(eye(3)+skew(r(4:6),1)+skew(r(4:6),1)^2/2+skew(r(4:6),1)^3/6+skew(r(4:6),1)^4/24);
%         f=(eye(3)+skew(r(7:9),1)+skew(r(7:9),1)^2/2+skew(r(7:9),1)^3/6+skew(r(7:9),1)^4/24);
        
        
        %     RX=(eye(3)+sin(norm(r(1:3)))/norm(r(1:3))*skew(r(1:3),1)+(1-cos(norm(r(1:3))))/norm(r(1:3))^2*skew(r(1:3),1)^2)*RX;
        %     RY=(eye(3)+sin(norm(r(4:6)))/norm(r(4:6))*skew(r(4:6),1)+(1-cos(norm(r(4:6))))/norm(r(4:6))^2*skew(r(4:6),1)^2)*RY;
        %     RZ=(eye(3)+sin(norm(r(7:9)))/norm(r(7:9))*skew(r(7:9),1)+(1-cos(norm(r(7:9))))/norm(r(7:9))^2*skew(r(7:9),1)^2)*RZ;
        
        
        %     if norm(r)<10^-3
        %     break
        %     end
        j=j+1;
        %         RX=(eye(3)+skew(r(1:3),1))*RX;
        %         RY=(eye(3)+skew(r(4:6),1))*RY;
        %         RZ=(eye(3)+skew(r(7:9),1))*RZ;
    end
end
xnum=[RX,RY,RZ,zeros(3,1),zeros(3,1),zeros(3,1)];



%d=norm(r);
%a=norm(RX'*RX0-eye(3));
%b=norm(RY'*RY0-eye(3));
%c=norm(RZ'*RZ0-eye(3));
end

