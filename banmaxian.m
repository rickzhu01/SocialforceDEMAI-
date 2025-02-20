% 创建一个空白的图像
figure;
hold on;

% % 设置马路的宽度和颜色
% road_width = 2;
% patch([0 road_width road_width 0], [0 0 10 10], [0 0 0]);

% 设置斑马线的宽度和间隔
zebra_width = 0.1;
zebra_gap = 0.1;

% 画出斑马线
for i = 0:20
    % 计算每条线的位置
    x = i * (zebra_width + zebra_gap);
    
    % 画出线条
    patch([x x x+zebra_width x+zebra_width], [20 22 22 20], 'w');
end

% 设置 x 轴的范围
xlim([0 road_width]);
axis equal;

% 关闭 hold on 状态
hold
