%绘制论文仿真实验中各部分误差在补偿前后随样本数量增大的对比图
clear
%% 绘制ERR1的对比图
ERR1_1=load('E:\毕业文件\博士论文\参数辨识章节\扫描仪采集数据\matlab Calibration comparison\10~300个样本数量的误差计算结果.mat','ERR1');
% ERR1_2=load('E:\毕业文件\博士论文\参数辨识章节\扫描仪采集数据\matlab Calibration comparison\10~300个样本数量的误差计算结果2.mat','ERR1');
% ERR1_3=load('E:\毕业文件\博士论文\参数辨识章节\扫描仪采集数据\matlab Calibration comparison\10~300个样本数量的误差计算结果3.mat','ERR1');
ERR1_4=load('E:\毕业文件\博士论文\参数辨识章节\扫描仪采集数据\matlab Calibration comparison\10~300个样本数量的误差计算结果4.mat','ERR1');
ERR1_mean=(ERR1_1.ERR1+ERR1_4.ERR1)/2;

ERR1_initial_1=load('E:\毕业文件\博士论文\参数辨识章节\扫描仪采集数据\matlab Calibration comparison\10~300个样本数量的误差计算结果.mat','ERR1_initial');
% ERR1_initial_2=load('E:\毕业文件\博士论文\参数辨识章节\扫描仪采集数据\matlab Calibration comparison\10~300个样本数量的误差计算结果2.mat','ERR1_initial');
% ERR1_initial_3=load('E:\毕业文件\博士论文\参数辨识章节\扫描仪采集数据\matlab Calibration comparison\10~300个样本数量的误差计算结果3.mat','ERR1_initial');
ERR1_initial_4=load('E:\毕业文件\博士论文\参数辨识章节\扫描仪采集数据\matlab Calibration comparison\10~300个样本数量的误差计算结果4.mat','ERR1_initial');
ERR1_initial_mean=(ERR1_initial_1.ERR1_initial+ERR1_initial_4.ERR1_initial)/2;
plot_nonzero_values(ERR1_initial_mean,ERR1_mean);
%% 绘制ERR2的对比图
ERR2_1=load('E:\毕业文件\博士论文\参数辨识章节\扫描仪采集数据\matlab Calibration comparison\10~300个样本数量的误差计算结果.mat','ERR2');
% ERR1_2=load('E:\毕业文件\博士论文\参数辨识章节\扫描仪采集数据\matlab Calibration comparison\10~300个样本数量的误差计算结果2.mat','ERR1');
% ERR1_3=load('E:\毕业文件\博士论文\参数辨识章节\扫描仪采集数据\matlab Calibration comparison\10~300个样本数量的误差计算结果3.mat','ERR1');
ERR2_4=load('E:\毕业文件\博士论文\参数辨识章节\扫描仪采集数据\matlab Calibration comparison\10~300个样本数量的误差计算结果4.mat','ERR2');
ERR2_mean=(ERR2_1.ERR2+ERR2_4.ERR2)/2;

ERR2_initial_1=load('E:\毕业文件\博士论文\参数辨识章节\扫描仪采集数据\matlab Calibration comparison\10~300个样本数量的误差计算结果.mat','ERR2_initial');
% ERR1_initial_2=load('E:\毕业文件\博士论文\参数辨识章节\扫描仪采集数据\matlab Calibration comparison\10~300个样本数量的误差计算结果2.mat','ERR1_initial');
% ERR1_initial_3=load('E:\毕业文件\博士论文\参数辨识章节\扫描仪采集数据\matlab Calibration comparison\10~300个样本数量的误差计算结果3.mat','ERR1_initial');
ERR2_initial_4=load('E:\毕业文件\博士论文\参数辨识章节\扫描仪采集数据\matlab Calibration comparison\10~300个样本数量的误差计算结果4.mat','ERR2_initial');
ERR2_initial_mean=(ERR2_initial_1.ERR2_initial+ERR2_initial_4.ERR2_initial)/2;
plot_nonzero_values(ERR2_initial_mean,ERR2_mean);
%% 绘制ERR3的对比图
ERR3_1=load('E:\毕业文件\博士论文\参数辨识章节\扫描仪采集数据\matlab Calibration comparison\10~300个样本数量的误差计算结果.mat','ERR3');
% ERR1_2=load('E:\毕业文件\博士论文\参数辨识章节\扫描仪采集数据\matlab Calibration comparison\10~300个样本数量的误差计算结果2.mat','ERR1');
% ERR1_3=load('E:\毕业文件\博士论文\参数辨识章节\扫描仪采集数据\matlab Calibration comparison\10~300个样本数量的误差计算结果3.mat','ERR1');
ERR3_4=load('E:\毕业文件\博士论文\参数辨识章节\扫描仪采集数据\matlab Calibration comparison\10~300个样本数量的误差计算结果4.mat','ERR3');
ERR3_mean=(ERR3_1.ERR3+ERR3_4.ERR3)/2;

ERR3_initial_1=load('E:\毕业文件\博士论文\参数辨识章节\扫描仪采集数据\matlab Calibration comparison\10~300个样本数量的误差计算结果.mat','ERR3_initial');
% ERR1_initial_2=load('E:\毕业文件\博士论文\参数辨识章节\扫描仪采集数据\matlab Calibration comparison\10~300个样本数量的误差计算结果2.mat','ERR1_initial');
% ERR1_initial_3=load('E:\毕业文件\博士论文\参数辨识章节\扫描仪采集数据\matlab Calibration comparison\10~300个样本数量的误差计算结果3.mat','ERR1_initial');
ERR3_initial_4=load('E:\毕业文件\博士论文\参数辨识章节\扫描仪采集数据\matlab Calibration comparison\10~300个样本数量的误差计算结果4.mat','ERR3_initial');
ERR3_initial_mean=(ERR3_initial_1.ERR3_initial+ERR3_initial_4.ERR3_initial)/2;
plot_nonzero_values(ERR3_initial_mean,ERR3_mean);

%% 绘制ERRQ的对比图
ERRQ_1=load('E:\毕业文件\博士论文\参数辨识章节\扫描仪采集数据\matlab Calibration comparison\10~300个样本数量的误差计算结果.mat','ERRQ');
% ERR1_2=load('E:\毕业文件\博士论文\参数辨识章节\扫描仪采集数据\matlab Calibration comparison\10~300个样本数量的误差计算结果2.mat','ERR1');
% ERR1_3=load('E:\毕业文件\博士论文\参数辨识章节\扫描仪采集数据\matlab Calibration comparison\10~300个样本数量的误差计算结果3.mat','ERR1');
ERRQ_4=load('E:\毕业文件\博士论文\参数辨识章节\扫描仪采集数据\matlab Calibration comparison\10~300个样本数量的误差计算结果4.mat','ERRQ');
ERRQ_mean=(ERRQ_1.ERRQ+ERRQ_4.ERRQ)/2;

ERRQ_initial_1=load('E:\毕业文件\博士论文\参数辨识章节\扫描仪采集数据\matlab Calibration comparison\10~300个样本数量的误差计算结果.mat','ERRQ_initial');
% ERR1_initial_2=load('E:\毕业文件\博士论文\参数辨识章节\扫描仪采集数据\matlab Calibration comparison\10~300个样本数量的误差计算结果2.mat','ERR1_initial');
% ERR1_initial_3=load('E:\毕业文件\博士论文\参数辨识章节\扫描仪采集数据\matlab Calibration comparison\10~300个样本数量的误差计算结果3.mat','ERR1_initial');
ERRQ_initial_4=load('E:\毕业文件\博士论文\参数辨识章节\扫描仪采集数据\matlab Calibration comparison\10~300个样本数量的误差计算结果4.mat','ERRQ_initial');
ERRQ_initial_mean=(ERRQ_initial_1.ERRQ_initial+ERRQ_initial_4.ERRQ_initial)/2;
plot_nonzero_values(ERRQ_initial_mean,ERRQ_mean);
