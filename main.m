clear all;
clc;

t = 15;
r = 10;                    %֡��,��λHz
B_collection = {};
P_collection = {};
edge1 = 1;
edge2 = 3;
bike = 0;       %����ʱ����ͨ�������г���
ped = 10;        %����ʱ����ͨ����������
lambda1 = ped / t;    %���˵���ʱ����
lambda2 = bike / t;   %�ǻ���������ʱ����
k = 1;             %�������г�����

%�������г�
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

%��������
for i=1:ped
    P_collection{end+1} = person;
end

%��ÿ�����г�������ֵ
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
    B_collection{1,i}.U(2,1) = 0.1;
    B_collection{1,i}.Profile = zeros(2,r*t);
    B_collection{1,i}.Profile(1,:) = 1.5 + rand(1,1);
    
    if isa(B_collection{1,i},'bicycle')
        B_collection{1,i}.Name = 'bicycle';
        B_collection{1,i}.m = weight() + normrnd(15, 2.150, [1, 1]);    %���������
        B_collection{1,i}.r = normrnd(0.85, 0.968, [1, 1]);     %�ߴ������
        B_collection{1,i}.Va_y = normrnd(3.51, 0.0324, [1, 1]);     %�ٶ������
        
    elseif isa(B_collection{1,i}, 'e_bike')
        B_collection{1,i}.Name = 'e_bike';
        B_collection{1,i}.m = weight() + normrnd(40, 6.449, [1, 1]);    %���������
        B_collection{1,i}.r = normrnd(0.87, 0.1254, [1, 1]);     %�ߴ������
        B_collection{1,i}.Va_y = normrnd(5.24, 0.102, [1, 1]);    %�����ٶ������
    end
    
end  
   
%��ÿ�����˵�����ֵ
for  i = 1 : length(P_collection)
    P_collection{1,i}.N = i ;
    if i >= 2
        P_collection{1,i}.t0 = P_collection{1,i-1}.t0 + timestart1(lambda1);
    else
        P_collection{1,i}.t0 = 0;
    end
    P_collection{1,i}.m = weight();         %�����������
    P_collection{1,i}.Va_x = normrnd(1.34, 0.287, [1, 1]);   %�����ٶ������
    
    P_collection{1,i}.Fa = zeros(2,r*t);
    P_collection{1,i}.Fb = zeros(2,r*t);
    P_collection{1,i}.Fc = zeros(2,r*t);
    P_collection{1,i}.F = zeros(2,r*t);
    P_collection{1,i}.A = zeros(2,r*t);
    P_collection{1,i}.U = zeros(2,r*t);
    P_collection{1,i}.U(2,1) = 0.1;
    P_collection{1,i}.Profile = zeros(2,r*t);
    P_collection{1,i}.Profile(2,:) = 20 + 2 * rand(1);   %��������
    
    
end
   
   
% ���濪ʼ
for i = 0.2 : 0.1 : t     %  time step 1/r second
    i
    i_r = int16(i*r);
    for j = 1 : length(P_collection)
        if i > P_collection{1,j}.t0
            oldspeed_x = P_collection{1,j}.U(1,i_r - 1);
            oldspeed_y = P_collection{1,j}.U(2,i_r - 1);
            [P_collection{1,j}.U(:,i_r),P_collection{1,j}.F(:,i_r),P_collection{1,j}.A(:,i_r),P_collection{1,j}.Fa(:,i_r),P_collection{1,j}.Fb(:,i_r),P_collection{1,j}.Fc(:,i_r)] = speed1(P_collection{1,j},B_collection, P_collection, i-0.1, edge1,edge2);
            %[P_collection{1,j}.Ux,P_collection{1,j}.Uy]=speed1(P_collection{1,j},B_collection, P_collection,i-0.1,edge1,edge2);
            P_collection{1,j}.Profile(:,i_r) = position1(oldspeed_x,oldspeed_y,P_collection{1,j},i-0.1);
        end
    end
    
    for j = 1 : length(B_collection)
        if i > B_collection{1,j}.t0
            oldspeed_x = B_collection{1,j}.U(1,i_r - 1);
            oldspeed_y = B_collection{1,j}.U(2,i_r - 1);
            if strcmp(B_collection{1,j}.Name, 'bicycle') 
                [B_collection{1,j}.U(:,i_r),B_collection{1,j}.F(:,i_r),B_collection{1,j}.A(:,i_r),B_collection{1,j}.Fa(:,i_r),B_collection{1,j}.Fb(:,i_r),B_collection{1,j}.Fc(:,i_r)] = speed2(B_collection{1,j},B_collection, P_collection, i-0.1, edge1, edge2);
                %[B_collection{1,j}.Ux,B_collection{1,j}.Uy]=speed2(B_collection{1,j},B_collection,P_collection,i-0.1,edge1,edge2);
                B_collection{1,j}.Profile(:,i_r) = position2(oldspeed_x,oldspeed_y,B_collection{1,j},i-0.1);
            else
                [B_collection{1,j}.U(:,i_r),B_collection{1,j}.F(:,i_r),B_collection{1,j}.A(:,i_r),B_collection{1,j}.Fa(:,i_r),B_collection{1,j}.Fb(:,i_r),B_collection{1,j}.Fc(:,i_r)] = speed3(B_collection{1,j},B_collection, P_collection, i-0.1, edge1, edge2);
                %[B_collection{1,j}.Ux,B_collection{1,j}.Uy]=speed2(B_collection{1,j},B_collection,P_collection,i-0.1,edge1,edge2);
                B_collection{1,j}.Profile(:,i_r) = position3(oldspeed_x,oldspeed_y,B_collection{1,j},i-0.1);
            
            end
            
        end
    end
    
end
 
timetext = uicontrol('style','text','string','0','fontsize',12,'position',[200,350,50,20]);%��ǰʱ��
writerObj = VideoWriter('test.avi'); %// ����һ����Ƶ�ļ������涯��
open(writerObj); %// �򿪸���Ƶ�ļ�

for i = 0.1 : 0.1 : t
    i_r = int16(r*i);
    set(gcf, 'Color', 'white');  % ����ǰͼ�εı�����ɫ����Ϊ��ɫ

    plot([0 4],[0 0]);%��������
    plot([0 0],[0 25]);%��������
    
 
    line([1,1],[0,19]);
    line([3,3],[0,19]);
    line([1,1],[23,25]);
    line([3,3],[23,25]);
    line([0,1],[19,19]);
    line([0,1],[23,23]);
    line([3,4],[19,19]);
    line([3,4],[23,23]);
    
    % �����ɫ��
    h_colorbar_a = colorbar('Position', [0.935, 0.1, 0.02, 0.8]);  % aֵ����ɫ��
    colormap(h_colorbar_a, ([linspace(1, 0, 256)', linspace(0, 0, 256)', linspace(0, 0, 256)']));  % ������ɫ��ӳ��Ϊ��[0,1,0]��[0,0,0]����ɫ
    %ylabel(h_colorbar_a, 'F-persons');
    h = ylabel(h_colorbar_a, 'F-persons');
    h.Position = h.Position + [-5 0 0];  % �����ƶ���ǩ
    ticks = [0, 100, 200, 300, 400];  % ���̶�ֵӳ�䵽[0, 1]��Χ
    tickLabels = {'0','100', '200','300', '400'};
    h_colorbar_a.Ticks = (ticks - min(ticks)) / (max(ticks) - min(ticks));
    h_colorbar_a.TickLabels = tickLabels;
    
%     h_colorbar_b = colorbar('Position', [0.75, 0.1, 0.02, 0.8]);  % bֵ����ɫ��
%     colormap(h_colorbar_b, ([linspace(0, 0, 256)', linspace(1, 0, 256)', linspace(0, 0, 256)']));  % ������ɫ��ӳ��Ϊ��[0,0,1]��[0,0,0]����ɫ
%     %ylabel(h_colorbar_b, 'F-bikes');
%     h = ylabel(h_colorbar_b, 'F-bikes');
%     h.Position = h.Position + [-5 0 0];  % �����ƶ���ǩ
%     tickLabels = {'0','250', '500','750', '1000'};
%     h_colorbar_b.Ticks = (ticks - min(ticks)) / (max(ticks) - min(ticks));
%     h_colorbar_b.TickLabels = tickLabels;
    
    
    
     % ���ð����ߵĿ�Ⱥͼ��
    zebra_width = 0.2;
    zebra_gap = 0.3;
        % ����������
    for k = 0:20
        % ����ÿ���ߵ�λ��
        x = k * (zebra_width + zebra_gap);
        % ��������
        patch([x x x+zebra_width x+zebra_width], [20 22 22 20], [0.5,0.5,0.5]);
    end
    hold on
    
    axis equal;
    axis([0 10 19 23]);%��ֹ����

    for j = 1:length(B_collection)
        if B_collection{1,j}.Profile(2,i_r)>0
            %scatter(B_collection{1,j}.Profile(1,i_10),B_collection{1,j}.Profile(2,i_10),'red','filled',"diamond");
            ecc = axes2ecc(0.7,0.08);  % ���ݳ�����Ͷ̰��������Բƫ����
            [elat,elon] = ellipse1(B_collection{1,j}.Profile(1,i_r),B_collection{1,j}.Profile(2,i_r),[0.7 ecc],90);
            c1 = sqrt(B_collection{1,j}.F(1,i_r)^2 + B_collection{1,j}.F(2,i_r)^2);
            c1_1 = mat2gray(c1, [0, 400]);    %��һ��
            plot(elat,elon,'color',[0 1-c1_1 0],'LineWidth',5);
            quiver(B_collection{1,j}.Profile(1,i_r),B_collection{1,j}.Profile(2,i_r),B_collection{1,j}.F(1,i_r)/(c1/2),B_collection{1,j}.F(2,i_r)/(c1/2),'color',[0 1-c1_1 0],'LineWidth',1);
            hold on
        end
    end
    for j = 1:length(P_collection)
        if P_collection{1,j}.Profile(1,i_r) > 0
            c2 = sqrt(P_collection{1,j}.F(1,i_r)^2 + P_collection{1,j}.F(2,i_r)^2);
            c2_1 = mat2gray(c2, [0, 100]);
            scatter(P_collection{1,j}.Profile(1,i_r),P_collection{1,j}.Profile(2,i_r),150,[1-c2_1,0,0],'filled');
            quiver(P_collection{1,j}.Profile(1,i_r),P_collection{1,j}.Profile(2,i_r),P_collection{1,j}.F(1,i_r)/c2/2,P_collection{1,j}.F(2,i_r)/c2/2,'color',[1-c2_1 0 0],'LineWidth',1,'ShowArrowHead','on');
            hold on
        end
    end
    
    hold off;
    frame = getframe(gcf);    %��ͼ�������Ƶ�ļ���
    writeVideo(writerObj,frame);    %��֡д����Ƶ
    %set(timetext,'string',i);
    str = [num2str(i), 's']; % ������ת��Ϊ�ַ�������ӵ�λ
    set(timetext, 'String', str); % ���� 'timetext' ���ַ���ֵ
    drawnow;
end

close(writerObj);
