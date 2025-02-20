data = xlsread('C:\Users\79124\Desktop\毕业论文\表格.xlsx','Sheet12','E1:E50');

% 步骤2: 使用mle函数来估计正态分布参数
params = mle(data, 'distribution', 'normal');
mu = params(1);  % 估计的均值
sigma = params(2);  % 估计的标准差

% 步骤3: 输出估计的参数
fprintf('估计的均值：%.2f\n', mu);
fprintf('估计的标准差：%.2f\n', sigma);

% 步骤4: 绘制数据的直方图和拟合的正态分布曲线
figure;
set(gcf, 'Color', 'white');  % 将当前图形的背景颜色设置为白色
%histogram(data, 30, 'Normalization', 'pdf');  % 绘制标准化的直方图
hold on;

% 计算正态分布的理论值
x = linspace(min(data), max(data), 1000);
pdf_norm = normpdf(x, mu, sigma);
[h, pValue, stat, cv] = jbtest(data);
plot(x, pdf_norm, 'r-', 'LineWidth', 2);  % 绘制正态分布的概率密度函数
histogram(data, 'NumBins', 6); 
title('数据的直方图与正态分布拟合');
xlabel('数据值');
ylabel('概率密度');
legend('直方图', '拟合的正态分布');

hold off;