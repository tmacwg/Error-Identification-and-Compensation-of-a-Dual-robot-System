clear
%��ȡ��־��,������ÿ���־�����תƽ�ƾ���B
for i=1:100
    Filename= ['E:\��ҵ�ļ�\����׫д\˫�����˱궨ʵ��\RefPts3D',num2str(i),'.asc'];
    B0((3*i-2):3*i,:)=load(Filename);
end
% [ B ] = Bcalculate( [B0(1:168,:);B0(202:300,:)] );
% [ B ] = Bcalculate( [B0(25:87,:)] );
 %[ B ] = Bcalculate( B0(1:150,:) );
 [ B ] = Bcalculate( [B0(1:150,:);B0(175:300,:)] );
 %��ȡkuka�����˲���,��������תƽ�ƾ���C
 C0=load('E:\��ҵ�ļ�\����׫д\˫�����˱궨ʵ��\kuka\dualrobot2350.txt');
%[ C] = Ccalculate( [C0(1:56,:);C0(68:100,:)] );
%[ C] = Ccalculate( [C0(9:29,:)] );
% [ C] = Ccalculate( C0(1:50,:) );
[ C] = Ccalculate( [C0(1:50,:);C0(59:100,:)] );
%��ȡABB�����˲���,��������תƽ�ƾ���A
A0=load('E:\��ҵ�ļ�\����׫д\˫�����˱궨ʵ��\abb\abb·����23.txt');
%[ A] = Acalculate( [A0(1:56,:);A0(68:100,:) ] );
%[ A] = Acalculate( [A0(9:29,:) ] );
% [ A] = Acalculate( A0(1:50,:) );
[ A] = Acalculate( [A0(1:50,:);A0(59:100,:)] );


