classdef person  %行人类
    %person are the object which run on roads

    %   此处显示详细说明
    
    properties    %变量名
        t0;       % entering time 
        N;        % Serial number of persons
        Profile;  % distance time profile
        Fa        % 自驱力
        Fb        % 道路使用者作用力
        Fc        % 边界作用力
        F;        % 合力
        A;        % 加速度
        U;        % 速度
        Ux;        % Instananeous Speed axis-X
        Uy;        % Instananeous Speed axis-Y
        q = 0.1;        % delta t
        Va_x ;     %期望速度
        Va_y = 0;        %期望速度 
        Ta = 0.5;          %个体适应时间
        Aa1 = 200;       %自行车对个体的作用力强度系数
        Aa2 = 50;        %行人对个体的作用力强度系数
        Ba1 = 6.2;       %自行车对个体的心理力作用范围
        Ba2 = 2;         %行人对行人作用力范围
        Bo = 1;          %个体对障碍物的心理作用范围
        r = 0.25;         %个体半径
        Ac = 100;        %障碍物排斥力强度系数
        m;          %个体重量
        V = 2;           %最大速度

        
        I = 2.7;             %惯性矩 1/2*m*r^2
        ko = 0.5;
        kd = 500;
        kl = 0.5;
        alpha = 0.3;
     
    end
      
  %  properties (Dependent)
         
  %  end
           

    methods
         
         function PE = person(t0,N,Profile,Fa,Fb,Fc,F,U,A,q,Va_x,Va_y,Ta,Aa1,Aa2,Ba1,Ba2,Bo,r,Ac,m,V,a,b,I,ko,kd,kl,alpha)%Construction functionr构造函数
             if nargin > 0
                 PE.t0 = t0;
                 PE.N = N;
                 PE.Profile = Profile;
                 PE.Fa = Fa;
                 PE.Fb = Fb;
                 PE.Fc = Fc;
                 PE.F = F;
                 PE.A = A;
                 PE.U = U;
                 PE.Ux = Ux;
                 PE.Uy = Uy;
                 PE.q = q;
                 PE.Va_x=Va_x;
                 PE.Va_y=Va_y;
                 PE.Ta=Ta;
                 PE.Aa1 = Aa1;
                 PE.Aa2 = Aa2;
                 PE.Ba1 = Ba1;
                 PE.Ba2 = Ba2;
                 PE.Bo=Bo;
                 PE.r=r;
                 PE.Ac=Ac;
                 PE.m=m;
                 PE.V=V;
                 
                 PE.I=I;
                 PE.ko=ko;
                 PE.kd=kd;
                 PE.kl=Kl;
                 PE.alpha=alpha;
             end
         end   
     
        %更新速度
        function [U,F,A,Fa,Fb,Fc] = speed1(obj,argu1,argu2,t,e1,e2)
            t_r = int16(10*t);
            t_r_old=int16(t_r - 1);
            
            Fa = zeros(2,1);
            Fb = zeros(2,1);
            Fc = zeros(2,1);
            F = zeros(2,1);
            A = zeros(2,1);
            U = zeros(2,1);
            
            %自驱力
            Fa(:,1) = [obj.m * (obj.Va_x - obj.U(1,t_r))/obj.Ta; obj.m*(obj.Va_y - obj.U(2,t_r))/obj.Ta];
            
            %道路使用者的作用力
            Fb(:,1) = 0;

%             for i = 1:length(argu1)
%                 if argu1{1,i}.Profile(1,t_r)<e2 && argu1{1,i}.Profile(1,t_r)>e1 && argu1{1,i}.Profile(2,t_r)~=0 && argu1{1,i}.Profile(1,t_r)>obj.Profile(1,t_r)
%                     R = sqrt((obj.Profile(1,t_r) - argu1{1,i}.Profile(1,t_r))^2 + (obj.Profile(2,t_r)-argu1{1,i}.Profile(2,t_r))^2);
%                     a = abs(argu1{1,i}.Profile(2,t_r) - obj.Profile(2,t_r));
%                     b = abs(argu1{1,i}.Profile(1,t_r) - obj.Profile(1,t_r));
%                     tan = a/b;
%                     if R>0 && R<obj.Ba1 && tan < 5000 
%                         fb = obj.Aa1 * exp((obj.r + argu1{1,i}.r - R)/obj.Ba1);
%                         Fb(:,1) = [(obj.Profile(1,t_r) - argu1{1,i}.Profile(1,t_r))/R * fb + Fb(1,1); (obj.Profile(2,t_r) - argu1{1,i}.Profile(2,t_r))/R * fb + Fb(2,1)];
%                     end
%                 end
%             end
            
            for j=1:length(argu2)
                if argu2{1,j}.Profile(1,t_r) > obj.Profile(1,t_r)
                    R = sqrt((obj.Profile(1,t_r) - argu2{1,j}.Profile(1,t_r))^2 + (obj.Profile(2,t_r)-argu2{1,j}.Profile(2,t_r))^2);
                    a = abs(argu2{1,j}.Profile(2,t_r) - obj.Profile(2,t_r));
                    b = abs(argu2{1,j}.Profile(1,t_r) - obj.Profile(1,t_r));
                    tan = a/b;
                    if R>0 && R<obj.Ba2 && tan < 1.732
                        fb = obj.Aa2 * exp((obj.r + argu2{1,j}.r - R)/obj.Ba2);
                        Fb(:,1) = [(obj.Profile(1,t_r)-argu2{1,j}.Profile(1,t_r))/R*fb+Fb(1,1);(obj.Profile(2,t_r)-argu2{1,j}.Profile(2,t_r))/R*fb+Fb(2,1)];
                    end
                end
            end
            
            %道路边界的作用力 
            Fc(:,1) = [0;obj.Ac * exp((obj.r - abs(obj.Profile(2,t_r) - (20-0.5)))/obj.Bo) - obj.Ac * exp((obj.r-abs(obj.Profile(2,t_r) - (22+0.5)))/obj.Bo)];
            
            %力和力矩的输入

            ktheta = obj.I * obj.kl * Fa(2,1);
            komega=obj.I*(1+obj.alpha)*sqrt(obj.kl*abs(Fa(2,1))/obj.alpha);
            Fu1 = obj.ko*(Fa(2,1) + Fb(2,1) + Fc(2,1)) - obj.kd * obj.U(2,t_r);        %法向力
            if (t_r>1)
                theta_i = atan2(obj.U(1,t_r),obj.U(2,t_r));                 %航向角1
                theta_i_old = atan2(obj.U(1,t_r_old),obj.U(2,t_r_old));     %航向角2
            else
                theta_i = atan(obj.U(1,t_r)/obj.U(2,t_r));
                theta_i_old = 0;
            end

            omega_i = (theta_i-theta_i_old)/obj.q;                   %角速度
            u_theta = -ktheta*(theta_i-0)-komega*omega_i;            %力矩
            Fu2 =-(u_theta/obj.I)*cos(theta_i);                      %力矩转为法向力的形式
            
            %计算合力
            F(:,1) = [Fa(1,1)+Fb(1,1)+Fc(1,1); Fu1 + Fu2];

            %计算加速度
            A(:,1) = [F(1,1)/obj.m; F(2,1)/obj.m];

            %计算速度
            U(2,1) =  obj.U(2,t_r) + A(2,1) * obj.q;
            if obj.U(1,t_r) + A(1,1) * obj.q >= 0
                U(1,1) = obj.U(1,t_r) + A(1,1) * obj.q;
            else
                U(1,1) = 0;
            end
        end

        %更新位置
        function Profile = position1(oldspeed_x,oldspeed_y,obj,t)
            t_r=int16(10*t);
            Profile = zeros(2,1);
            %if obj.Profile(2,t_r)+(oldspeed_y+obj.U(2,t_r))/2*obj.q>19.5 && obj.Profile(2,t_r)+(oldspeed_y+obj.U(2,t_r))/2*obj.q<22.5
                Profile = [obj.Profile(1,t_r)+(oldspeed_x+obj.U(1,t_r))/2*obj.q; obj.Profile(2,t_r)+(oldspeed_y+obj.U(2,t_r))/2*obj.q];
        end
        
    end
    
end
