%================================================================
% ���ܣ�   ��������Է�����ͨ���е��ؽ� dK*dQ=dPe
% ������        dK��dPe, dQ���۵Ĺؽ����        
% ����ֵ��       CaldqT ����Ĺؽ�������� 
%               CaldqT  ����Ĺؽ�������  
%               Caldb   �����{S-B}΢���������
% ��Ҫ˼·��
% ��ע��    
% ���÷�����
% ���ڣ�    2014/9/10 
%================================================================  

%%%%%%1.%ȥ���������������ĵ�һ���ؽ�dq1
[RdK,jbdK]=rref(dK);
%%�ҳ�1��Ӧ������
njbdK=size(jbdK,2);
RjbdK=zeros(njbdK,1);
for i=1:size(jbdK,2)
    for j=1:size(dK,1)
         if RdK(j,jbdK(:,i))==1
             RjbdK(i,1)=j;%%%�ҳ������޹�������Ԫ��"1"��Ӧ��������Ĭ��Ϊ1->njbdK
             break;
         end
        
    end
end
dq1=dK(:,jbdK)\dPe;


%%%%%%2.���㲻ȥ��������������һ���ؽ� CaldQ
ndq1=size(dq1,1);
dq2=zeros(ndq1,1);
Desdq=[DesdqR;Desdb'];
for i=1:ndq1
    dq2(i,:)=dq1(i,:)-RdK(RjbdK(i,:),:)*Desdq+Desdq(jbdK(:,i),1);%����� CaldQ�а���dq1������ֵ
end
CaldQ=Desdq;
for i=1:njbdK
    CaldQ(jbdK(:,i),:)=dq2(i,:);%%��� CaldQ�а���dq1������ֵΪdq2,���������ı���ֵ��������ֵDesdq����Ӧ�ı���
end

%%%%%%3.����Ӧ�Ĺؽڲ���������CaldqT��{S-B}΢�����ʸ��Caldb
DesdqT=dQ;
CaldqT=[CaldQ(7:12,:),CaldQ(1:6,:),CaldQ(19:24,:),CaldQ(13:18,:),CaldQ(25:30,:)];
Desdb=Desdq(31:36,:);
Caldb=CaldQ(31:36,:);


%%%%%%%4.��֤
dtdq=CaldQ-Desdq;
dtdb=Caldb-Desdb;
dtdQT=CaldqT-DesdqT;













%s=5;
    
