i=48;
p=3;

A=abs(P_collection{1, p}.Profile(1,i)-B_collection{1, 1}.Profile(1,i));
B=abs(P_collection{1, p}.Profile(2,i)-B_collection{1, 1}.Profile(2,i));
C=sqrt(B^2+A^2);

p1=(A-0.6-0.25)/U_p
p2=(A+0.6+0.25)/U_p

b1=(B-0.9-0.25)/U_b
b2=(B+0.9+0.25)/U_b

U_p=P_collection{1, p}.U(1,i);
U_b=B_collection{1, 1}.U(2,i);%让行前速度
V=(A/C)*U_p+(B/C)*U_b;  %相对速度
B %让行前距离

if (p1+p2)<(b1+b2)
    ttc=(C-0.25-0.9)/V   %行人先到
else 
    ttc=(C-0.25-0.9)/V   %自行车先到

% 假设 data 是你的表格，第一行是 x 坐标，第二行是 y 坐标
x = B_collection{1, 1}.Profile(1, 1:96); % 提取第一行作为 x 坐标
y = B_collection{1, 1}.Profile(2, 1:96); % 提取第二行作为 y 坐标
set(gcf, 'Color', 'white');  % 将当前图形的背景颜色设置为白色
plot(y, -x, '-o', 'MarkerSize', 3); % 画出散点图并连线
title('非机动车轨迹坐标图'); % 添加标题
xlabel('y 坐标'); % 添加 x 轴标签
ylabel('x 坐标'); % 添加 y 轴标签
axis equal;
xlim([0 25]); % 设置 x 轴的范围为 0 到 4
ylim([-4 0]); % 设置 x 轴的范围为 0 到 4

figure;
% 假设 data 是你的表格，第一行是 x 坐标，第二行是 y 坐标
x = P_collection{1, p}.Profile(1, 1:96); % 提取第一行作为 x 坐标
y = P_collection{1, p}.Profile(2, 1:96); % 提取第二行作为 y 坐标
set(gcf, 'Color', 'white');  % 将当前图形的背景颜色设置为白色
plot(y, -x, '-o', 'MarkerSize', 3); % 画出散点图并连线
title('行人轨迹坐标图'); % 添加标题
xlabel('y 坐标'); % 添加 x 轴标签
ylabel('x 坐标'); % 添加 y 轴标签
axis equal;
xlim([0 25]); % 设置 x 轴的范围为 0 到 4
ylim([-4 0]); % 设置 x 轴的范围为 0 到 4

figure;
set(gcf, 'Color', 'white');  % 将当前图形的背景颜色设置为白色
plot(B_collection{1, 1}.U(1,1:73),'-o', 'MarkerSize', 3);
figure;
set(gcf, 'Color', 'white');  % 将当前图形的背景颜色设置为白色
plot(B_collection{1, 1}.U(2,1:73),'-o', 'MarkerSize', 3);
