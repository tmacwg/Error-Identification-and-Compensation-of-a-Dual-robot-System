%================================================================
% ���ܣ�   �����˱궨
% ������   Ps1��Ps2 ��������̬����������n����������ɨ�����µ�����[x y z]'��ά��3Xn
%          E1��E2   n�����������̬ e=[x y z q0 q1 q2 q3]' ά��7Xn
%          Q        �����ؽڵĹؽڲ���   [alpha a theta d beta]
%          matAng1��matAng2 n��λ���¶�Ӧ�Ĺؽڽ�
% ����ֵ��  dqΪ���ؽڵĲ���ֵ
% ��Ҫ˼·��������ֵTbs���ٲ���
% ��ע��    
% ���÷�����
% ���ڣ�    2014/8/23 20:37
%================================================================
Ps1=simPs(1:numPt/2,:)';  Ps2=simPs(numPt/2+1:numPt,:)'; %3*n
E1=MeaTse(:,:,1:numPt/2);   E2=MeaTse(:,:,numPt/2+1:numPt);%7*n
matAng1=matAng(1:numPt/2,:);matAng2=matAng(numPt/2+1:numPt,:);
%%%%%%%%%%%%%%%%%%%%%%%%
  
%%%%%%%%����1���������˻�����ϵ��ɨ����֮���λ��Tbs%%%%%%%%%%%%%
%b��ʾ������ϵ s��ʾɨ��������ϵ

nPt=size(Ps1,2);%�������

%����1.1 ���dM��dPbe
dM=zeros(3*nPt,12);%��ʼ��dM��С
dPbe=zeros(3*nPt,1);%��ʼ��dPbe��С
for i=1:nPt
    T1i=inv(E1(:,:,i));
    T2i=inv(E2(:,:,i));%�����ڲ�����i��������ʱ��������ĩ����Ի�����ϵ��λ�˶�
    M1i=[kron(Ps1(:,i)',T1i(1:3,1:3)),T1i(1:3,1:3)];%�����һ����̬�µ�M����
    M2i=[kron(Ps2(:,i)',T2i(1:3,1:3)),T2i(1:3,1:3)];%����ڶ�����̬�µ�M����
   
    P1bei=T1i(1:3,4);%���������ĩ������ڻ�����ϵ����̬
    P2bei=T2i(1:3,4);
   
    dM(3*i-2:3*i,:)=M1i-M2i;
    dPbe(3*i-2:3*i,1)=P2bei-P1bei;
end

%����1.2 ��С�������Tbs
X=inv(dM'*dM)*dM'*dPbe;
invTbs=[X(1:3,1),X(4:6,1),X(7:9,1),X(10:12,1);0 0 0 1];
MyTbs=invTbs;
tmpTbs=inv(invTbs);
tmpTbs=T2StdT(tmpTbs);%תΪ��׼��������

%%%%%%%%%%%%% START                         ����badTsb
% dTbs=[0.1,0.2,0.3,0.4,0.5,0.6]/1000;
% badtbs=tbs-[1 2 4]*0.1/1000;
% badRpy=Rpy-[2 5 6]*0.1/1000;
% badTsb=[RPY2R(badRpy),badtbs';0 0 0 1];%����Ϊ�趨��Tbs
%badTsb=Tbs; %����ʵ��Tbs 
%%%%%%%%%%%%%  END

%%%%%%%%%%% START                         ������۵� dTbs  
dT=tmpTbs\(DesTbs-tmpTbs);
%%%%���б�׼baddT�ľ���Tsb
dT2=D2T(T2D(dT));
MeaTbs=DesTbs/((eye(4)+dT2));
Desdb=T2D(dT2)';
%%%%%%%%%%%  END


% %%%%%%%%����1���˹����������˻�����ϵ��ɨ����֮���λ��Tbs�����%%%%%%%%%%%%%
% nPt=size(Ps1,2);%�������
% k=2*rand(3,1)-[1,1,1]';
% k=k/norm(k);
% R=DesTbs(1:3,1:3)*rot(k,0.2);
% k=k*0.001;
% t=DesTbs(1:3,4)+k;
% MeaTbs=[R,t;[0 0 0 1]];
% 
% dT=MeaTbs\(DesTbs-MeaTbs);
% dT2=D2T(T2D(dT));
% Desdb=T2D(dT)';
% %%%%%%%%%%%  END







%%%%%%%%%%%%%%%%%����2�������˱궨 ���dq%%%%%%%%%%%%%%%%%%%%%
%����2.1 �����˽�ģ
% clear IRB4400
% IRB4400=RobotModel2(Q);
%����2.2 ���dK��dPe
num_dq=36;%������������
dK=zeros(3*nPt,num_dq);%��ʼ��dK
dPe=zeros(3*nPt,1);%��ʼ��dPe
for i=1:nPt
    %����Ki1  Ki2  Ki1-Ki2
    %����dP
    T1i=inv(MeaTbs*E1(:,:,i));
    T2i=inv(MeaTbs*E2(:,:,i));%�����ڲ�����i��������ʱ��������ĩ����Ի�����ϵ��λ�˶�
    Theta1i=matAng1(i,:); %���˶�ѧ����һ����̬�Ĺؽڱ���
    Theta2i=matAng2(i,:);%���˶�ѧ���ڶ�����̬�Ĺؽڱ���
    Q1i=[Q(:,1:2),Theta1i',Q(:,4:5)];%���¹ؽڲ���
    Q2i=[Q(:,1:2),Theta2i',Q(:,4:5)];%���¹ؼ�����
    K1i=CalK(Q1i,MeaTbs,Ps1(:,i));  %����K1
    K2i=CalK(Q2i,MeaTbs,Ps2(:,i));  %����K2
    Pe1i=T1i(1:3,1:3)*Ps1(:,i)+T1i(1:3,4);%�����i���������ڻ�����ĩ������ϵe�µ�����1
    Pe2i=T2i(1:3,1:3)*Ps2(:,i)+T2i(1:3,4);%�����i���������ڻ�����ĩ������ϵe�µ�����2
    dK(3*i-2:3*i,:)=K1i-K2i; %����dK
    dPe(3*i-2:3*i,1)=Pe2i-Pe1i; %������������̬�£������������ĩ������ϵe�µ������dP 
end
dK(:,18)=0;
s=rank(dK);
m=0;
%����2.3 ��С�������ؽ�Q�Ĳ�������dq
%%%%%%%%%%%%%%  START
 %�ⷨ1 
 %rag=rank(AG); rdG=rank(dG);raK=rank(AK);rdK=rank(dK);�ж��Ƿ�Ϊ���Ⱦ���
 %dq=inv(dK'*dK)*dK'*dPe;
 %�ⷨ2 %��Solution)_dQ.m
%%%%%%%%%%%%%%  END 
