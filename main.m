
%%=============================   =============================      =============================         
clear all;
clc;
% Parameters  General simulation  
t = 15;                    %   time  
r = 10;                    %   time  resolution 
B_collection = {};
P_collection = {};
edge1 = 1;     % 
edge2 = 3;
bike = 3;       %  number of bikes   
ped = 6;        % number of pedestrians
lambda1 = ped / t;    %   parameter for generation   
lambda2 = bike / t;   %    parameter for generation   
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
    %  Generate bike agents 
for i = 1:bike
    alphabet = [0, 1];
    prob = [k, 1-k];
    randomNumber = randsrc(1, 1, [alphabet; prob]);
    if randomNumber == 0
        B_collection{1,i} = bicycle;   %bicycle or e_bike
    else 
        B_collection{1,i} = e_bike;
    end
end

%   =================  add from the end 
for i=1:ped
    P_collection{end+1} = person;
end

%  Generate bike agents 
for  i = 1 : length(B_collection)
    B_collection{1,i}.N = i ; 
    if i >= 2
        B_collection{1,i}.t0 = B_collection{1,i-1}.t0 + timestart2(lambda2);
    else
        B_collection{1,i}.t0 = 0;
    end
    B_collection{1,i}.F = zeros(2,r*t);
    B_collection{1,i}.A = zeros(2,r*t);
    B_collection{1,i}.Fa = zeros(2,r*t);
    B_collection{1,i}.Fb = zeros(2,r*t);
    B_collection{1,i}.Fc = zeros(2,r*t);
    B_collection{1,i}.U = zeros(2,r*t);
    B_collection{1,i}.U(2,1) = 0.2;
    B_collection{1,i}.Profile = zeros(2,r*t);
    B_collection{1,i}.Profile(1,:) = 1.5 + rand(1,1);
    
    if isa(B_collection{1,i},'bicycle')
        B_collection{1,i}.Name = 'bicycle';
        B_collection{1,i}.m = weight() + normrnd(15, 2.150, [1, 1]);    %体重随机化
        B_collection{1,i}.r = normrnd(0.85, 0.968, [1, 1]);     %尺寸随机化
        B_collection{1,i}.Va_y = normrnd(3.51, 0.0324, [1, 1]);     %速度随机化
        
    elseif isa(B_collection{1,i}, 'e_bike')
        B_collection{1,i}.Name = 'e_bike';
        B_collection{1,i}.m = weight() + normrnd(40, 6.449, [1, 1]);    %体重随机化
        B_collection{1,i}.r = normrnd(0.87, 0.1254, [1, 1]);     %尺寸随机化
        B_collection{1,i}.Va_y = normrnd(5.24, 0.102, [1, 1]);    %期望速度随机化
    end
    
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
    P_collection{1,i}.Profile(2,:) = 20 + 5* rand(1);   %行人生成
    P_collection{1,i}.destination_x = 10;
    P_collection{1,i}.destination_y = 20 + 2*rand(1);
    P_collection{1,i}.density = zeros(1,r*t);
    for j  = 1:r*t
        P_collection{1,i}.attention(1,j) = P_collection{1,i}.attention(1,1); %行人初始的注意力机制均为20
    end
    P_collection{1,i}.attention_count = zeros(1,r*t);   %行人初始的注意力计数器初始化
end
   
 %==============================  ==============================   ==============================  
%for i= 1:length(P_collection)
      
%     len (P_collection{1, i}.Profile , P_collection{1, 1}.Profile )    % ===
         % sort by distance 
   % for  j= 1:length(P_collection) 
         %   point2vector(P_collection{1, j}.Profile  ,P_collection{1, i}.Profile, (P_collection{1, 1}.Profile)
    %end 
%end 

% Simulation loop

step_leap = 20;    %    Simulation step leap ===================================
m=0; 
step_sim= 0.1; 


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
            [P_collection{1,j}.U(:,step),P_collection{1,j}.F(:,step),P_collection{1,j}.A(:,step),P_collection{1,j}.Fa(:,step),P_collection{1,j}.Fb(:,step),P_collection{1,j}.Fc(:,step)] = speed1(P_collection{1,j},B_collection, P_collection, n-step_sim, edge1,edge2);
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
            [P_collection{1,j}.U(:,step),P_collection{1,j}.F(:,step),P_collection{1,j}.A(:,step),P_collection{1,j}.Fa(:,step),P_collection{1,j}.Fb(:,step),P_collection{1,j}.Fc(:,step)] = speed1(P_collection{1,j},B_collection, P_collection, n-step_sim, edge1,edge2);
            %[P_collection{1,j}.Ux,P_collection{1,j}.Uy]=speed1(P_collection{1,j},B_collection, P_collection,i-0.1,edge1,edge2);    
            P_collection{1,j}.Profile(:,step) = position1(oldspeed_x,oldspeed_y,P_collection{1,j},n-step_sim); %  input  (oldspeed_x,oldspeed_y,obj,t)
         end 
    end
    
    for j = 1 : length(P_collection)
        if P_collection{1,j}.attention_count(1,step-1)+1 == P_collection{1,j}.attention(1,step)
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
       
        
    for j = 1 : length(B_collection)   % Bicycle
        if i > B_collection{1,j}.t0
            oldspeed_x = B_collection{1,j}.U(1,step - 1);
            oldspeed_y = B_collection{1,j}.U(2,step - 1);
            if strcmp(B_collection{1,j}.Name, 'bicycle') 
                [B_collection{1,j}.U(:,step ),B_collection{1,j}.F(:,step ),B_collection{1,j}.A(:,step ),B_collection{1,j}.Fa(:,step ),B_collection{1,j}.Fb(:,step ),B_collection{1,j}.Fc(:,step )] = speed2(B_collection{1,j},B_collection, P_collection, n-step_sim, edge1, edge2);
                %[B_collection{1,j}.Ux,B_collection{1,j}.Uy]=speed2(B_collection{1,j},B_collection,P_collection,i-0.1,edge1,edge2);
                B_collection{1,j}.Profile(:,step ) = position2(oldspeed_x,oldspeed_y,B_collection{1,j}, n-step_sim);
            else
                [B_collection{1,j}.U(:,step ),B_collection{1,j}.F(:,step ),B_collection{1,j}.A(:,step ),B_collection{1,j}.Fa(:,step ),B_collection{1,j}.Fb(:,step ),B_collection{1,j}.Fc(:,step )] = speed3(B_collection{1,j},B_collection, P_collection,  n-step_sim, edge1, edge2);
                %[B_collection{1,j}.Ux,B_collection{1,j}.Uy]=speed2(B_collection{1,j},B_collection,P_collection,i-0.1,edge1,edge2);
                B_collection{1,j}.Profile(:,step ) = position3(oldspeed_x,oldspeed_y,B_collection{1,j}, n-step_sim);
            
            end
        end
    end

    for j = 1 : length(P_collection)
        P_collection{j}.density(1,step) =  density(P_collection{j},P_collection,step);
    end
end
 
timetext = uicontrol('style','text','string','0','fontsize',12,'position',[200,350,50,20]);%当前时间
writerObj = VideoWriter('test.avi'); %// 定义一个视频文件用来存动画
open(writerObj); %// 打开该视频文件

for i = 0.1 : 0.1 : t
   step = int16(r*i);
    set(gcf, 'Color', 'white');  % 将当前图形的背景颜色设置为白色

    plot([0 4],[0 0]);%画坐标轴
    plot([0 0],[0 25]);%画坐标轴

    line([1,1],[0,19]);
    line([3,3],[0,19]);
    line([1,1],[23,25]);
    line([3,3],[23,25]);
    line([0,1],[19,19]);
    line([0,1],[23,23]);
    line([3,4],[19,19]);
    line([3,4],[23,23]);
    
    % 添加颜色条
    h_colorbar_a = colorbar('Position', [0.65, 0.1, 0.02, 0.8]);  % a值的颜色条
    colormap(h_colorbar_a, ([linspace(1, 0, 256)', linspace(0, 0, 256)', linspace(0, 0, 256)']));  % 设置颜色条映射为从[0,1,0]到[0,0,0]的绿色
    %ylabel(h_colorbar_a, 'F-persons');
    h = ylabel(h_colorbar_a, 'F-persons');
    h.Position = h.Position + [-5 0 0];  % 向左移动标签
    ticks = [0, 100, 200, 300, 400];  % 将刻度值映射到[0, 1]范围
    tickLabels = {'0','100', '200','300', '400'};
    h_colorbar_a.Ticks = (ticks - min(ticks)) / (max(ticks) - min(ticks));
    h_colorbar_a.TickLabels = tickLabels;
    
    h_colorbar_b = colorbar('Position', [0.75, 0.1, 0.02, 0.8]);  % b值的颜色条
    colormap(h_colorbar_b, ([linspace(0, 0, 256)', linspace(1, 0, 256)', linspace(0, 0, 256)']));  % 设置颜色条映射为从[0,0,1]到[0,0,0]的蓝色
    %ylabel(h_colorbar_b, 'F-bikes');
    h = ylabel(h_colorbar_b, 'F-bikes');
    h.Position = h.Position + [-5 0 0];  % 向左移动标签
    tickLabels = {'0','250', '500','750', '1000'};
    h_colorbar_b.Ticks = (ticks - min(ticks)) / (max(ticks) - min(ticks));
    h_colorbar_b.TickLabels = tickLabels;
    
    
    
     % 设置斑马线的宽度和间隔
    zebra_width = 0.2;
    zebra_gap = 0.3;
        % 画出斑马线
    for k = 0:10
        % 计算每条线的位置
        x = k * (zebra_width + zebra_gap);
        % 画出线条
        patch([x x x+zebra_width x+zebra_width], [20 22 22 20], 'black');
    end
    hold on
    
    axis equal;
    axis([0 4 0 25]);%防止抖动

    for j = 1:length(B_collection)
        if B_collection{1,j}.Profile(2,step )>0
            %scatter(B_collection{1,j}.Profile(1,i_10),B_collection{1,j}.Profile(2,i_10),'red','filled',"diamond");
            ecc = axes2ecc(0.7,0.08);  % 根据长半轴和短半轴计算椭圆偏心率
            [elat,elon] = ellipse1(B_collection{1,j}.Profile(1,step ),B_collection{1,j}.Profile(2,step ),[0.7 ecc],90);
            c1 = sqrt(B_collection{1,j}.F(1,step )^2 + B_collection{1,j}.F(2,step )^2);
            c1_1 = mat2gray(c1, [0, 400]);    %归一化
            plot(elat,elon,'color',[0 1-c1_1 0],'LineWidth',5);
            quiver(B_collection{1,j}.Profile(1,step ),B_collection{1,j}.Profile(2,step ),B_collection{1,j}.F(1,step )/(c1/2),B_collection{1,j}.F(2,step )/(c1/2),'color',[0 1-c1_1 0],'LineWidth',1);
            hold on
        end
    end
    for j = 1:length(P_collection)
        if P_collection{1,j}.Profile(1,step ) > 0
            c2 = sqrt(P_collection{1,j}.F(1,step )^2 + P_collection{1,j}.F(2,step )^2);
            c2_1 = mat2gray(c2, [0, 100]);
            scatter(P_collection{1,j}.Profile(1,step ),P_collection{1,j}.Profile(2,step ),50,[1-c2_1,0,0],'filled');
            quiver(P_collection{1,j}.Profile(1,step ),P_collection{1,j}.Profile(2,step ),P_collection{1,j}.F(1,step )/c2,P_collection{1,j}.F(2,step )/c2,'color',[1-c2_1 0 0],'LineWidth',1,'ShowArrowHead','on');
            hold on
        end
    end
    
    hold off;
    frame = getframe(gcf);    %把图像存入视频文件中
    writeVideo(writerObj,frame);    %将帧写入视频
    %set(timetext,'string',i);
    str = [num2str(i), 's']; % 将数字转换为字符串并添加单位
    set(timetext, 'String', str); % 更新 'timetext' 的字符串值
    drawnow;
end

close(writerObj);