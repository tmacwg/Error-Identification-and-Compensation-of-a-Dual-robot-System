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
% E1=DesTse(:,:,1:numPt/2);   E2=DesTse(:,:,numPt/2+1:numPt);%7*n
matAng1=matAng(1:numPt/2,:);matAng2=matAng(numPt/2+1:numPt,:);
M1=DesTo1e1(:,:,1:numPt/2);  M2=DesTo1e1(:,:,numPt/2+1:numPt);
%%%%%%%%%%%%%%%%%%%%%%%%
  
% %%%%%%%%����1���������˻�����ϵ��ɨ����֮���λ��Tbs%%%%%%%%%%%%%
% %b��ʾ������ϵ s��ʾɨ��������ϵ
% 
% nPt=size(Ps1,2);%�������
% 
% %����1.1 ���dM��dPbe
% dM=zeros(3*nPt,12);%��ʼ��dM��С
% dPbe=zeros(3*nPt,1);%��ʼ��dPbe��С
% for i=1:nPt
%     T1i=inv(E1(:,:,i));
%     T2i=inv(E2(:,:,i));%�����ڲ�����i��������ʱ��������ĩ����Ի�����ϵ��λ�˶�
%     M1i=[kron(Ps1(:,i)',T1i(1:3,1:3)),T1i(1:3,1:3)];%�����һ����̬�µ�M����
%     M2i=[kron(Ps2(:,i)',T2i(1:3,1:3)),T2i(1:3,1:3)];%����ڶ�����̬�µ�M����
%    
%     P1bei=T1i(1:3,4);%���������ĩ������ڻ�����ϵ����̬
%     P2bei=T2i(1:3,4);
%    
%     dM(3*i-2:3*i,:)=M1i-M2i;
%     dPbe(3*i-2:3*i,1)=P2bei-P1bei;
% end
% 
% %����1.2 ��С�������Tbs
% X=inv(dM'*dM)*dM'*dPbe;
% invTbs=[X(1:3,1),X(4:6,1),X(7:9,1),X(10:12,1);0 0 0 1];
% MyTbs=invTbs;
% tmpTbs=inv(invTbs);
% tmpTbs=T2StdT(tmpTbs);%תΪ��׼��������
% 
% %%%%%%%%%%%%% START                         ����badTsb
% % dTbs=[0.1,0.2,0.3,0.4,0.5,0.6]/1000;
% % badtbs=tbs-[1 2 4]*0.1/1000;
% % badRpy=Rpy-[2 5 6]*0.1/1000;
% % badTsb=[RPY2R(badRpy),badtbs';0 0 0 1];%����Ϊ�趨��Tbs
% %badTsb=Tbs; %����ʵ��Tbs 
% %%%%%%%%%%%%%  END
% 
% %%%%%%%%%%% START                         ������۵� dTbs  
% dT=tmpTbs\(DesTbs-tmpTbs);
% %%%%���б�׼baddT�ľ���Tsb
% dT2=D2T(T2D(dT));
% MeaTbs=DesTbs/((eye(4)+dT2));
% Desdb=T2D(dT2)';
% %%%%%%%%%%%  END


%%%%%%%%����1���˹���������ĩ�˱�ϵ��ɨ����֮���λ��Te1s�����%%%%%%%%%%%%%
nPt=size(Ps1,2);%�������
k=2*rand(3,1)-[1,1,1]';
k=k/norm(k);
R=DesTe1s(1:3,1:3)*rot(k,0.01);
k=k*0.0001;
t=DesTe1s(1:3,4)+k;
MeaTe1s=[R,t;[0 0 0 1]];

dT=MeaTe1s\(DesTe1s-MeaTe1s);
dT2=D2T(T2D(dT));
De1s=T2D(dT)';
%%%%%%%%%%%  END

%%%%%%%%����2���˹�������������ϵ��ӹ�������ĩ��֮���λ��Tte2�����%%%%%%%%%%%%%
nPt=size(Ps1,2);%�������
k=2*rand(3,1)-[1,1,1]';
k=k/norm(k);
R=DesTte2(1:3,1:3)*rot(k,0.01);
k=k*0.0001;
t=DesTte2(1:3,4)+k;
MeaTte2=[R,t;[0 0 0 1]];

dT=MeaTte2\(DesTte2-MeaTte2);
dT2=D2T(T2D(dT));
Dte2=T2D(dT)';
%%%%%%%%%%%  END


%%%%%%%%����3���˹������ӹ������˻�����ϵ�����������ϵ֮���λ��To2o1�����%%%%%%%%%%%%%
nPt=size(Ps1,2);%�������
k=2*rand(3,1)-[1,1,1]';
k=k/norm(k);
R=DesTo2o1(1:3,1:3)*rot(k,0.01);
k=k*0.0001;
t=DesTo2o1(1:3,4)+k;
MeaTo2o1=[R,t;[0 0 0 1]];

dT=MeaTo2o1\(DesTo2o1-MeaTo2o1);
dT2=D2T(T2D(dT));
Do2o1=T2D(dT)';
%%%%%%%%%%%  END







%%%%%%%%%%%%%%%%%����2�������˱궨 ���dq%%%%%%%%%%%%%%%%%%%%%
%����2.1 �����˽�ģ
% clear IRB4400
% IRB4400=RobotModel2(Q);
%����2.2 ���dK��dPe
num_dq=48;%������������
dK=zeros(3*nPt,num_dq);%��ʼ��dK
dPe=zeros(3*nPt,1);%��ʼ��dPe
for i=1:nPt
    %����Ki1  Ki2  Ki1-Ki2
    %����dP
    T1i=inv(MeaTe1s*M1(:,:,i)*MeaTo2o1*E1(:,:,i)*MeaTte2);
    T2i=inv(MeaTe1s*M2(:,:,i)*MeaTo2o1*E2(:,:,i)*MeaTte2);%�����ڲ�����i��������ʱ��������ĩ�����ɨ��������ϵ��λ�˶�

    To2s1=MeaTe1s*M1(:,:,i)*MeaTo2o1;
    To2s2=MeaTe1s*M2(:,:,i)*MeaTo2o1;
    Te1s=MeaTe1s;
    Tts1=MeaTe1s*M1(:,:,i)*MeaTo2o1*E1(:,:,i)*MeaTte2;
    Tts2=MeaTe1s*M2(:,:,i)*MeaTo2o1*E2(:,:,i)*MeaTte2;
    
    
%     MeaTts=(DesTe1s*DesTo1e1(:,:,i)*DesTo2o1*DesTse(:,:,i))*DesTte2;
%     MeaTst=MeaTts;
    
    Theta1i=matAng1(i,:); %���˶�ѧ����һ����̬�Ĺؽڱ���
    Theta2i=matAng2(i,:);%���˶�ѧ���ڶ�����̬�Ĺؽڱ���
    Q1i=[Q(:,1:2),Theta1i',Q(:,4:5)];%���¹ؽڲ���
    Q2i=[Q(:,1:2),Theta2i',Q(:,4:5)];%���¹ؼ�����
    K1i=CalK_Dualrobot(Q1i,To2s1,Te1s,Tts1,Ps1(:,i));  %����K1
    K2i=CalK_Dualrobot(Q2i,To2s2,Te1s,Tts2,Ps2(:,i));  %����K2
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
