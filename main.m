
%%=============================   =============================      =============================         
clear all;
clc;
% Parameters  General simulation  
t = 50;                    %   time  
r = 10;                    %   time  resolution 
P_collection = {};
edge1 = 1;     % 
edge2 = 3;

ped = 20;        % number of pedestrians
lambda1 = ped / t;    %   parameter for generation   

k = 1;             %     Portion 

maxn = 1e5 + 5;
eps = 1e-8;
%%=============================   =============================      =============================            
% Parameters  DEM 
%num_particles = 10;  %=====================
% mass = 50;             % Mass of each particle (kg)  %=====================
radius = 0.05;          % Radius of particles (m)
k_n = 1e4;              % Normal stiffness (N/m)
damping = 0.1;          % Damping coefficient

dt = 1e-4;              % Time step (s)
total_time = 12.0;       % Total simulation time (s)

%能量损失函数记录 
energyLoss = zeros(1, r*t);

%% 2D 

p1= [1, 1];
p2= [4, 4];
p0= [2, 4];
C=cross2D( p1(1,1 ),p1(1,2 ),p2(1,1 ),p2(1,2 )); 
%point2vector(p0,p1,p2)
% Angle between two vectors   
OA=[1,1];
OB=[1,-1];
AngleDegree(OA,OB)
% Generate random pedestrain coordination  in a rectangle area
profile = unifrnd(1,8, ped,2 );
Destination = [0.5, 9];

%  test 
for i = 1:ped   
    
   for j= 1:ped   
       A= Destination-profile(i,:); 
       B= profile(j,:)- profile(i,:); 
       visual_degree_table(i,j) =AngleDegree(A,B);
   end

end 

%   =================  add from the end 
for i=1:ped
    P_collection{end+1} = person;
end
   
%Generate peds agents 
for  i = 1 : length(P_collection)
    P_collection{1,i}.N = i ;
    if i >= 2
        P_collection{1,i}.t0 = P_collection{1,i-1}.t0 + timestart1(lambda1);   % entry time  exprnd()
    else
        P_collection{1,i}.t0 = 0;
    end
    P_collection{1,i}.m = weight();         %质量随机化，
    P_collection{1,i}.Va_x = normrnd(1.34, 0.287, [1, 1]);   %期望速度随机化
    P_collection{1,i}.Va_y = 0; %初始状态下竖直方向上速度为0
    P_collection{1,i}.Fa = zeros(2,r*t);
    P_collection{1,i}.Fb = zeros(2,r*t);
    P_collection{1,i}.Fc = zeros(2,r*t);
    P_collection{1,i}.F = zeros(2,r*t);
    P_collection{1,i}.A = zeros(2,r*t);
    P_collection{1,i}.U = zeros(2,r*t);
    P_collection{1,i}.nforces= zeros(2,r*t);
    P_collection{1,i}.U(2,1) = 0.1;       %=============================         
    P_collection{1,i}.Profile = zeros(2,r*t); 
    P_collection{1,i}.Profile(2,:) = 1 + 6* rand(1);   %行人生成
    P_collection{1,i}.destination_x = 60;
    P_collection{1,i}.destination_y = 2 + 4*rand(1);
    P_collection{1,i}.density = zeros(1,r*t);
    P_collection{1,i}.coeffFa_log   = zeros(1,r*t);
    P_collection{1,i}.coeffFb_log   = zeros(1,r*t);
    P_collection{1,i}.coeffFc_log   = zeros(1,r*t);
    P_collection{1,i}.attention_log = zeros(1,r*t);
    P_collection{1,i}.lambda = randi([1 9]) / 10;

    for j  = 1:r*t
        P_collection{1,i}.attention(1,j) = P_collection{1,i}.attention(1,1); %行人初始的注意力机制均为20
        P_collection{1,i}.coeffFa(1,j) = P_collection{1,i}.coeffFa(1,1);
        P_collection{1,i}.coeffFb(1,j) = P_collection{1,i}.coeffFb(1,1);
        P_collection{1,i}.coeffFc(1,j) = P_collection{1,i}.coeffFc(1,1);
    end
    P_collection{1,i}.attention_count = zeros(1,r*t);   %行人初始的注意力计数器初始化
end

step_leap = 20;    %    Simulation step leap ===================================
m=0; 
step_sim= 0.1; 

%% ---------- RL 初始化（共享策略网络） ----------
stateDim   = 3;      % [ρ, Ec, v]
actionDim  = 4;      % [coeffFa, coeffFb, coeffFc, attention]

lr     = 1e-3;
gamma  = 0.90;

layers = [ ...
    featureInputLayer(stateDim,"Name","state")
    fullyConnectedLayer(16,"Name","fc1")
    reluLayer("Name","relu1")
    fullyConnectedLayer(actionDim,"Name","fc2")
    sigmoidLayer("Name","sig") ];
policyNetCell  = cell(1,ped);
for k = 1:ped
    policyNetCell{k} = dlnetwork(layerGraph(layers));
end

% 每个行人各自缓存 1-step TD 所需数据
lastStateCell  = cell(1,ped);
lastActionCell = cell(1,ped);

% ――――― 监视器 ―――――
tpm = trainingProgressMonitor;
tpm.Info    = "Step";
tpm.Metrics = ["Reward","AvgReward","rho","attention"];
tpm.XLabel  = "Environment Step";

avgWindow   = 200;                % 滑动平均窗口
rewardQueue = zeros(1,avgWindow);

% Main simulation  

for n  = 0.2 : step_sim : t        %  time step 1/r second

    step = int16(n*r);
    m=m+1;

     % Pedestrians  Particle  interactions
    for j = 1 : length(P_collection)
        now_attention_count=P_collection{1,j}.attention_count(1,step-1)+1;
        last_attention_count=P_collection{1,j}.attention_count(1,step-1);

         if   P_collection{1,j}.t0-n <2                                 %  initialization of social force model  tine elapse
            oldspeed_x = P_collection{1,j}.U(1,step - 1);    
            oldspeed_y = P_collection{1,j}.U(2,step - 1);
            [P_collection{1,j}.U(:,step),P_collection{1,j}.F(:,step),P_collection{1,j}.A(:,step),P_collection{1,j}.Fa(:,step),P_collection{1,j}.Fb(:,step),P_collection{1,j}.Fc(:,step)] = speed1(P_collection{1,j}, P_collection, n-step_sim, edge1,edge2);
            P_collection{1,j}.Profile(:,step) = position1(oldspeed_x,oldspeed_y,P_collection{1,j},n-step_sim); %  input  (oldspeed_x,oldspeed_y,obj,t)
            P_collection{1,j}.attention_count(1,step)=P_collection{1,j}.attention_count(1,step-1)+1;
             
         elseif    P_collection{1,j}.t0-n>=2 && now_attention_count<P_collection{1,j}.attention(1,step)           %  keep previous social force model data 
            P_collection{1,j}.F(:,step)=P_collection{1,j}.F(:,step-1)  ;  
            P_collection{1,j}.A(:,step)=P_collection{1,j}.A(:,step-1) ; 
            P_collection{1,j}.Fa(:,step)=P_collection{1,j}.Fa(:,step-1) ;
            P_collection{1,j}.Fb(:,step)=P_collection{1,j}.Fb(:,step-1) ;
            P_collection{1,j}.Fc(:,step)=P_collection{1,j}.Fc(:,step-1);
            P_collection{1,j}.U(:,step) = P_collection{1,j}.U(:,step -1);
            oldspeed_x = P_collection{1,j}.U(1,step);
            oldspeed_y = P_collection{1,j}.U(2,step);
            P_collection{1,j}.Profile(:,step) = position1(oldspeed_x,oldspeed_y,P_collection{1,j},n-step_sim); %  input  (oldspeed_x,oldspeed_y,obj,t)
            P_collection{1,j}.attention_count(1,step)=P_collection{1,j}.attention_count(1,step-1)+1;

         elseif    P_collection{1,j}.t0-n>=2 && now_attention_count==P_collection{1,j}.attention(1,step)  %      time leap  
            oldspeed_x = P_collection{1,j}.U(1,step - 1);
            oldspeed_y = P_collection{1,j}.U(2,step - 1);
            [P_collection{1,j}.U(:,step),P_collection{1,j}.F(:,step),P_collection{1,j}.A(:,step),P_collection{1,j}.Fa(:,step),P_collection{1,j}.Fb(:,step),P_collection{1,j}.Fc(:,step)] = speed1(P_collection{1,j}, P_collection, n-step_sim, edge1,edge2);
            P_collection{1,j}.Profile(:,step) = position1(oldspeed_x,oldspeed_y,P_collection{1,j},n-step_sim); %  input  (oldspeed_x,oldspeed_y,obj,t)
         end 
    end
    
    for j = 1 : length(P_collection)
        if P_collection{1,j}.attention_count(1,step-1)+1 == P_collection{1,j}.attention(1,step)
            P_collection{1,j}.attention_count(1,step)=0;
        elseif P_collection{1,j}.attention_count(1,step-1)+1 >= P_collection{1,j}.attention(1,step)
            P_collection{1,j}.attention_count(1,step)=0;
        end
    end

       % Particle interactions DEM 
     for i = 1: length(P_collection)-1
        for j = i+1: length(P_collection)
                     
            delta = P_collection{1,j}.Profile(:,step) - P_collection{1,i}.Profile(:,step);
            dist = norm(delta);
            overlap = 2 * radius - dist;
            
             if overlap > 0
                % Normal force (spring-dashpot model)
                normal = delta / dist;
                rel_vel =  P_collection{1,j}.U(:,step) - P_collection{1,i}.U(:,step);  %=== caution
                force_n = k_n * overlap * normal - damping * rel_vel;
                
               P_collection{1,i}.nforces(:,step)  = P_collection{1,i}.nforces(:,step) - force_n;  %==========
               P_collection{1,j}.nforces(:,step)  = P_collection{1,j}.nforces(:,step) + force_n;  
               
              % Update velocities and positions
               mass= P_collection{1,j}.m ; 
               P_collection{1,j}.U(:,step) = P_collection{1,j}.U(:,step) + (P_collection{1,j}.nforces(:,step) ./ mass) .*[dt;dt] ; % ====  intergration  person class   body mass  class 
               P_collection{1,j}.Profile(:,step)=  position1( P_collection{1,j}.U(1,step), P_collection{1,j}.U(2,step ),P_collection{1,j},n-step_sim);  % ==========intergration person class    update position 
                  
             end
          end 
     end 

     [energyLoss(step), pairLossStep] = collisionEnergyLoss( P_collection, step, k_n, damping, dt, radius);

    for j = 1 : length(P_collection)
        P_collection{j}.density(1,step) =  density(P_collection{j},P_collection,step);
    end
    
    %% ---------- RL 互动（所有行人，共享网络） ----------
    % 为本环境步预创建暂存器（NaN 方便忽略未参与者）
    stepReward =  nan(1,length(P_collection));
    stepRho    =  nan(1,length(P_collection));
    stepAtt    =  nan(1,length(P_collection));
    
    for j = 1:length(P_collection)
    
        % 仅当行人已进入通道才决策
        if n < P_collection{j}.t0
            continue
        end
    
        % ===== 1. 当前状态 =====
        rho_now = P_collection{j}.density(1,step);
        Ec_now  = energyLoss(step);
        v_now   = norm(P_collection{j}.U(:,step));
        s_t     = dlarray([rho_now; Ec_now; v_now],"CB");
    
        % ===== 2. 策略网络输出动作 (0,1) =====
        a_t = predict(policyNetCell{j}, s_t);
    
        coeffFa_new   = 0.5 + 1.5 * a_t(1);
        coeffFb_new   = 0.5 + 1.5 * a_t(2);
        coeffFc_new   = 0.5 + 1.5 * a_t(3);
        attention_new = 40  - 30  * a_t(4);    % 10–40
    
        % ===== 3. 把动作写回未来时间轴 =====
        P_collection{j}.coeffFa(1,step+1:end)   = coeffFa_new;
        P_collection{j}.coeffFb(1,step+1:end)   = coeffFb_new;
        P_collection{j}.coeffFc(1,step+1:end)   = coeffFc_new;
        P_collection{j}.attention(1,step+1:end) = attention_new;
    
        % ===== 4. 记录日志（便于后处理、绘图） =====
        P_collection{j}.coeffFa_log(1,step)    = coeffFa_new;
        P_collection{j}.coeffFb_log(1,step)    = coeffFb_new;
        P_collection{j}.coeffFc_log(1,step)    = coeffFc_new;
        P_collection{j}.attention_log(1,step)  = attention_new;
    
        % ===== 5. 奖励函数（始终计算，以便监视器使用） =====
        speedErr = abs(v_now - P_collection{j}.Va_x);
        w_v      = 5;                % 速度误差权重
        w_rho    = 15;               % ρ-attention 耦合权重
        attNorm  = attention_new/40; % 0.25–1
    
        reward_t = - Ec_now ...
                   - w_v  * speedErr ...
                   - w_rho* rho_now * attNorm;
    
        % === 5-b 仅当有上一步缓存时才做 TD 更新 ===
        if ~isempty(lastStateCell{j})
            target = reward_t;       % TD(0)
    
            lossFun = @(net) mseLoss(net, lastStateCell{j}, lastActionCell{j}, target);

            [grad,~] = dlfeval(lossFun, policyNetCell{j});
            policyNetCell{j} = dlupdate(@(w,g) w - lr*g, policyNetCell{j}, grad);

        end
    
        % ===== 6. 缓存本步状态 / 动作 =====
        lastStateCell{j}  = s_t;
        lastActionCell{j} = a_t;
    
        % ===== 7. 写入暂存器，用 NaN 规避 “未进入” 行人 =====
        stepReward(j) = reward_t;
        stepRho(j)    = rho_now;
        stepAtt(j)    = attention_new;
    
    end  % ← for-j
    
    % ===== 8. 环境步结束后，统一写可视化监视器 =====
    meanRwd = mean(stepReward,"omitnan");
    meanRho = mean(stepRho,   "omitnan");
    meanAtt = mean(stepAtt,   "omitnan");
    
    % 更新滑动平均
    rewardQueue(mod(step-1,avgWindow)+1) = meanRwd;
    avgRwd = mean(rewardQueue(rewardQueue~=0));
    
    updateInfo(tpm,"Step",string(step));
    recordMetrics(tpm, step, ...
        Reward    = meanRwd, ...
        AvgReward = avgRwd, ...
        rho       = meanRho, ...
        attention = meanAtt);
end
 
% —————— 1. 准备 Figure 和 UI ——————
figure;
set(gcf,'Color','white');
ax = gca;
hold(ax,'on');

% ① 固定坐标系（防止自动缩放）
xlim(ax, [0 60]);
ylim(ax, [0 8]);
axis(ax,'equal');           % 锁定长宽比
ax.XLimMode = 'manual';    
ax.YLimMode = 'manual';

% ② 画出矩形通道的边界
%    [x y width height]
rectangle(ax, 'Position',[0, 0, 60, 8], 'EdgeColor','k', 'LineWidth',1);

% ③ 横向颜色条（下方正中）
hCb = colorbar(ax,'southoutside');
colormap(ax, [linspace(1,0,256)', zeros(256,1), zeros(256,1)]);  % 红→黑
hCb.Position = [0.15 0.09 0.70 0.03];
xlabel(hCb,'F-persons');
ticks = [0 100 200 300 400];
hCb.Ticks      = (ticks - min(ticks)) / (max(ticks) - min(ticks));
hCb.TickLabels = arrayfun(@num2str, ticks, 'UniformOutput',false);

% ④ 时间文本（颜色条正下方）
timetext = uicontrol('Style','text', ...
    'Units','normalized', ...
    'String','0.0 s', ...
    'FontSize',12, ...
    'BackgroundColor','white', ...
    'Position',[0.45 0.04 0.10 0.03]);

% —————— 2. 打开视频写入器 ——————
writerObj = VideoWriter('test.avi');
open(writerObj);

% —————— 3. 主循环：绘制 + 写帧 ——————
for i = 0.1 : 0.1 : t
    step = int16(r * i);

    % 清除上一帧的散点和箭头
    delete(findall(ax, 'Type','Scatter'));
    delete(findall(ax, 'Type','Quiver'));

    % 绘制所有粒子和力向量
    for j = 1:length(P_collection)
        pos = P_collection{1,j}.Profile(:,step);
        if pos(1) > 0
            F = P_collection{1,j}.F(:,step);
            c2 = norm(F);
            c2_1 = mat2gray(c2, [0 100]);
            scatter(ax, pos(1), pos(2), 50, [1-c2_1,0,0], 'filled');
            quiver(ax, pos(1), pos(2), F(1)/c2, F(2)/c2, ...
                   'Color',[1-c2_1,0,0], 'LineWidth',1, 'MaxHeadSize',0.5);
        end
    end

    % 写入当前帧
    frame = getframe(gcf);
    writeVideo(writerObj, frame);

    % 更新时间文本
    set(timetext, 'String', sprintf('%.1f s', i));
    drawnow;
end

% —————— 4. 关闭视频 ——————
close(writerObj);
