% 输入4组数据的平均误差和标准偏差值
data1 = [0.27, 0.18]; %本文方法
data2 = [0.71, 0.35];  %初始标定方法
data3 = [0.61, 0.13]; %Wu的方法
data4 = [0.44, 0.21]; %Wang的方法

% 将数据存储在一个矩阵中
data = [data1; data2; data3; data4];
set(0,'defaultfigurecolor','w');
% 绘制柱状图
figure;
b=bar(data(:,1));

b.FaceColor = 'flat';
b.CData(1,:) = [0  255 0];
b.CData(2,:) = [200  255 0];
b.CData(3,:) = [0  255 255];
b.CData(4,:) = [255  0 71];


% 添加标签和标题

xlabel('Method','fontsize',14,'FontName','Times New Roman');
ylabel('Error (mm)','fontsize',14,'FontName','Times New Roman');
% title('四组数据的平均误差');

% 添加误差条
hold on;
errorbar(data(:,1), data(:,2), 'k', 'LineStyle', 'none', 'LineWidth', 1.5);
hold off;
