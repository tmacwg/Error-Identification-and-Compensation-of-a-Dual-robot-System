clear
%读取标志点,并计算每组标志点的旋转平移矩阵B
for i=1:100
    Filename= ['E:\毕业文件\论文撰写\双机器人标定实验\RefPts3D',num2str(i),'.asc'];
    B0((3*i-2):3*i,:)=load(Filename);
end
% [ B ] = Bcalculate( [B0(1:168,:);B0(202:300,:)] );
% [ B ] = Bcalculate( [B0(25:87,:)] );
 %[ B ] = Bcalculate( B0(1:150,:) );
 [ B ] = Bcalculate( [B0(1:150,:);B0(175:300,:)] );
 %读取kuka机器人参数,并计算旋转平移矩阵C
 C0=load('E:\毕业文件\论文撰写\双机器人标定实验\kuka\dualrobot2350.txt');
%[ C] = Ccalculate( [C0(1:56,:);C0(68:100,:)] );
%[ C] = Ccalculate( [C0(9:29,:)] );
% [ C] = Ccalculate( C0(1:50,:) );
[ C] = Ccalculate( [C0(1:50,:);C0(59:100,:)] );
%读取ABB机器人参数,并计算旋转平移矩阵A
A0=load('E:\毕业文件\论文撰写\双机器人标定实验\abb\abb路径点23.txt');
%[ A] = Acalculate( [A0(1:56,:);A0(68:100,:) ] );
%[ A] = Acalculate( [A0(9:29,:) ] );
% [ A] = Acalculate( A0(1:50,:) );
[ A] = Acalculate( [A0(1:50,:);A0(59:100,:)] );


