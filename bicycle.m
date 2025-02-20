classdef bicycle  %自行车类
    %bikes are the object which run on roads

    %   此处显示详细说明
    
    properties      %变量名
        t0;         % entering time 
        Name;
        N;          % Serial number of persons
        Profile;    % distance time profile
        Fa          % 自驱力
        Fb          % 道路使用者作用力
        Fc          % 边界作用力
        F;          % 合力
        A;          % 加速度
        U;          % 速度
        q = 0.1;      % delta t
        Va_x = 0;     %期望速度
        Va_y;     %期望速度 
        Ta = 1;       %个体反应时间
        Aa1 = 300;     %自行车对个体的作用力强度系数
        Aa2 = 500;     %行人对个体的作用力强度系数
        Ba1 = 4.0;       %个体的心理力作用范围
        Ba2 = 5.2;
        r = 0.9;      %个体半径
        Ao = 150;     %障碍物排斥力强度系数
        Bo = 1;       %个体对障碍物的心理作用范围
        m ;       %骑自行车人的重量
        V = 5;        %最大速度
        
        I = 15.75;             %惯性矩 Π/4*a*b^3
        ko = 1;
        kd = 500;
        kl = 0;
        alpha = 3;
        
     
    end
      
  %  properties (Dependent)
         
  %  end
           

    methods
        function AM = bicycle(t0,N,Profile,Fa,Fb,Fc,F,U,A,q,Ta,Aa1,Aa2,Ba,Bo,r,Ao,m,V,I,ko,kd,kl,alpha)%Construction functionr构造函数
            if nargin > 0
                AM.t0 = t0;
                AM.Name = bicycle;
                AM.N = N;
                AM.Profile = Profile;
                AM.Fa = Fa;
                AM.Fb = Fb;
                AM.Fc = Fc;
                AM.F = F;
                AM.A = A;
                AM.U = U;
                AM.Ux = Ux;
                AM.Uy = Uy;
                AM.q = q;
                AM.Ta = Ta;
                AM.Aa1 = Aa1;
                AM.Aa2 = Aa2;
                AM.Ba = Ba;
                AM.Bo = Bo;
                AM.r = r;
                AM.Ao = Ao;
                AM.m = m;
                AM.V = V;

                
                AM.I = I;
                AM.ko = ko;
                AM.kd = kd;
                AM.kl = kl;
                AM.alpha = alpha;
            end
        end
     
          %更新速度  argu1=自行车集合；argu2=行人集合
        function [U,F,A,Fa,Fb,Fc] = speed2(obj,argu1,argu2,t,e1,e2)
            t_r = int16(10*t);
            t_r_old = int16(t_r - 1);
            
            Fa = zeros(2,1);
            Fb = zeros(2,1);
            Fc = zeros(2,1);
            F = zeros(2,1);
            A = zeros(2,1);
            U = zeros(2,1);
            
            %自驱力
            Fa(:,1) = [obj.m*(obj.Va_x - obj.U(1,t_r))/obj.Ta; obj.m*(obj.Va_y - obj.U(2,t_r))/obj.Ta];
                        
            %道路使用者的作用力
            Fb(:,1) = 0;

            for i = 1 : length(argu1)
                if argu1{1,i}.Profile(1,t_r)<e2 && argu1{1,i}.Profile(1,t_r)>e1 && obj.Profile(2,t_r)<argu1{1,i}.Profile(2,t_r)
                    R = sqrt((obj.Profile(1,t_r) - argu1{1,i}.Profile(1,t_r))^2 + (obj.Profile(2,t_r) - argu1{1,i}.Profile(2,t_r))^2);
                    a = abs(obj.Profile(2,t_r) - argu1{1,i}.Profile(2,t_r));%纵坐标差
                    b = abs(obj.Profile(1,t_r) - argu1{1,i}.Profile(1,t_r));%横坐标差
                    tan = b/a;
                    if R>0 && R<obj.Ba1 && tan < 1.732
                        fb = obj.Aa1 * exp((obj.r + argu1{1,i}.r - R)/obj.Ba1);
                        Fb(:,1) = [(obj.Profile(1,t_r) - argu1{1,i}.Profile(1,t_r))/R * fb + Fb(1,1);(obj.Profile(2,t_r) - argu1{1,i}.Profile(2,t_r))/R * fb + Fb(2,1)];
                    end
                end
            end
            
            for j = 1:length(argu2)
                if argu2{1,j}.Profile(1,t_r)>e1 && argu2{1,j}.Profile(1,t_r)<e2 && obj.Profile(2,t_r)<argu2{1,j}.Profile(2,t_r)
                    R = sqrt((obj.Profile(1,t_r)-argu2{1,j}.Profile(1,t_r))^2+(obj.Profile(2,t_r)-argu2{1,j}.Profile(2,t_r))^2);
                    a = abs(obj.Profile(2,t_r) - argu2{1,i}.Profile(2,t_r));%纵坐标差
                    b = abs(obj.Profile(1,t_r) - argu2{1,i}.Profile(1,t_r));%横坐标差
                    tan = b/a;
                    if R>0 && R<obj.Ba2 && tan<1.732 
                        fb = obj.Aa2 * exp((obj.r + argu2{1,j}.r - R)/obj.Ba2);
                        Fb(:,1) = [(obj.Profile(1,t_r) - argu2{1,j}.Profile(1,t_r))/R * fb+Fb(1,1);(obj.Profile(2,t_r) - argu2{1,j}.Profile(2,t_r))/R * fb + Fb(2,1)];
                    end
                end
            end
            
            %道路边界的作用力
            Fc(:,1) = [obj.Ao * exp((obj.r - abs(obj.Profile(1,t_r) - e1))/obj.Bo) - obj.Ao * exp((obj.r - abs(obj.Profile(1,t_r) - e2))/obj.Bo); 0];
            
            %力和力矩的输入

            ktheta = obj.I * obj.kl * Fa(1,1);
            komega = obj.I * (1 + obj.alpha) * sqrt(obj.kl * abs(Fa(1,1))/obj.alpha);
            Fu1 = obj.ko * (Fa(1,1) + Fb(1,1) + Fc(1,1)) - obj.kd * obj.U(1,t_r);        %法向力
            if (t_r>1)
                theta_i = atan2(obj.U(1,t_r),obj.U(2,t_r));                 %航向角1
                theta_i_old = atan2(obj.U(1,t_r_old),obj.U(2,t_r_old));     %航向角2
            else
                theta_i = atan(obj.U(1,t_r) / obj.U(2,t_r));
                theta_i_old = 0;
            end

            omega_i = (theta_i - theta_i_old) / obj.q;                      %角速度
            u_theta = -ktheta * (theta_i - 0) - komega * omega_i;            %力矩
            Fu2 = -(u_theta / obj.I) * cos(theta_i);                      %力矩转为法向力的形式
            
            %计算合力
            F(:,1) = [Fu1 + Fu2; Fa(2,1) + Fb(2,1) + Fc(2,1)];
            
            %计算加速度
            A(:,1) = [F(1,1)/obj.m; F(2,1)/obj.m];
           
            %计算速度
            U(1,1) = obj.U(1,t_r) + A(1,1) * obj.q;
            if obj.U(2,t_r) + A(2,1) * obj.q >= 0
                U(2,1) = obj.U(2,t_r) + A(2,1) * obj.q;
            else
                U(2,1) = 0;
            end
        end

        %更新位置
        function Profile = position2(oldspeed_x,oldspeed_y,obj,t)
            t_r = int16(10*t);
            Profile = zeros(2,1);
            Profile = [obj.Profile(1,t_r) + ((oldspeed_x + obj.U(1,t_r))/2) * obj.q; obj.Profile(2,t_r) + ((oldspeed_y + obj.U(2,t_r))/2) * obj.q];
        end
       
    end
    
end
