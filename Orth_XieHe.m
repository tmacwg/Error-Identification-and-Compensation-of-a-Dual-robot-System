function [ B ] = Orth_XieHe( A )
%����л�˵ľ�������������������ת����A�ı�׼��������B
%   ���룺A��������������ת����
%   �����B�������������ת����
AA=Schmidt(A);
OulerAngle_A = R2RPY(AA);

for i=1:5
C=[[0 0 0],-AA(3,:),-AA(2,:);AA(3,:),[0 0 0],-AA(1,:);-AA(2,:),AA(1,:),[0 0 0]]';
D=[(A(1,1)-AA(1,1)),(A(1,2)-AA(1,2)),(A(1,3)-AA(1,3)),(A(2,1)-AA(2,1)),(A(2,2)-AA(2,2)),(A(2,3)-AA(2,3)),(A(3,1)-AA(3,1)),(A(3,2)-AA(3,2)),(A(3,3)-AA(3,3))]';
delta=inv(C'*C)*C'*D;
OulerAngle_A=OulerAngle_A+delta';
AA=RPY2R([OulerAngle_A(1),OulerAngle_A(2),OulerAngle_A(3)]);

end

B=AA;

end

