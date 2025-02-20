% 读取Excel文件
%data = xlsread('C:\Users\79124\Desktop\毕业论文\表格.xlsx','Sheet12','A1:A46');
filename = 'C:\Users\79124\Desktop\毕业论文\表格.xlsx';
table = readtable(filename, 'Sheet', 'sheet12', 'Range', 'A1:A46');


% 将表格数据转换为向量
data = table2array(table);

% 创建直方图
figure; % 创建一个新的图形窗口
set(gcf, 'Color', 'white');  % 将当前图形的背景颜色设置为白色
histogram(data, 'Normalization', 'count'); % 创建直方图

% 添加标题和轴标签
title('TTC分布直方图');
xlabel('Two-Dimensional TTC');
ylabel('频数');

