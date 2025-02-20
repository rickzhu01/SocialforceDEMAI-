% 定义区间
interval1 = [0.4270, 1.2852];
interval2 = [0.6351, 1.0839];

% 创建一个新的图形窗口
figure;
set(gcf, 'Color', 'white');  % 将当前图形的背景颜色设置为白色
% 使用红色线条绘制第一个区间，并设置透明度
patch([interval1 fliplr(interval1)], [0 0 0.1 0.1], 'r', 'FaceAlpha', 0.5, 'EdgeColor', 'none'); hold on;

% 使用蓝色线条绘制第二个区间，并设置透明度
patch([interval2 fliplr(interval2)], [0.1 0.1 0.2 0.2], 'b', 'FaceAlpha', 0.5, 'EdgeColor', 'none');

% 添加图例
legend('行人', '非机动车');

% 添加标题和坐标轴标签
title('时间重合');
xlabel('t');
ylabel('Intervals');
axis equal;
xlim([0 2]); % 设置 x 轴的范围为 -1 到 5
ylim([0 0.3]); % 设置 x 轴的范围为 -1 到 5

% 不显示y轴的刻度值
yticks([]);


% 显示网格grid on;
