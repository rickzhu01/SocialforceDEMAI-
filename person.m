classdef person  %行人类
    %person are the object which run on roads
    
    properties
        t0;       % entering time 
        N;        % Serial number of persons
        Profile;  % distance time profile
        Fa        % self driving force 
        Fb        %  agent repulsion force 
        Fc        %  border force 
        F;        % 
        A;        % Acceleraioon 
        U;        %  speed
        Ux;        % Instananeous Speed axis-X
        Uy;        % Instananeous Speed axis-Y
        q = 0.1;        % delta t  ====================================
        Va_x ;     % Expected speed 
        Va_y;        % Expected speed 
        Ta = 0.5;          %  accustomed time 
        Aa1 = 200;       %  bicycle to pedestrian
        Aa2 = 50;        %   pedestrian to pedestrian
        Ba1 = 6.2;       %      bicycle to pedestrian   effective  range
        Ba2 = 3;         %  pedestrian to pedestrian   effective range
        Bo = 1;          %  pedestrian to obstacle  effective   range 
        r = 0.25;         %  radius of pedestrian 
        Ac = 100;        %  border  repulsion
        m;          %  ??  ===========
        orentation;          %================================================== Attention decides the  orentation coordination 
                
        V = 2;           % Max speed 

        I = 2.7;             %   oment of inertia of an area  1/2*m*r^2
        ko = 0.5;
        kd = 500;
        kl = 0.5;
        alpha = 0.3;
       
        nforces;
        
        attention = 20; %注意力机制
        attention_count = 0;
        destination_x;
        destination_y;
        density;
        coeffFa = 1;
        coeffFb = 1;
        coeffFc = 1;
        
        % ===== 记录 RL 每一步学到的新系数（便于论文后处理） =====
        coeffFa_log     % 1×(r*t)  动态 Fa 系数
        coeffFb_log     % 1×(r*t)  动态 Fb 系数
        coeffFc_log     % 1×(r*t)  动态 Fc 系数
        attention_log   % 1×(r*t)  动态 attention
        
        % 新增各向异性参数 改进基础的社会力模型 
        % 为了让每个行人的行走风格不同，我们在main中随机化赋值lambda。
        % 0 = 最大各向异性，只看前方；1 = 各向同性（原模型）
        lambda;
    end
      
        
    methods
         
         function PE = person(t0,N,Profile,Fa,Fb,Fc,F,U,A,q,Va_x,Va_y,Ta,Aa1,Aa2,Ba1,Ba2,Bo,r,Ac,m,V,a,b,I,ko,kd,kl,alpha)%Construction functionr鏋勯€犲嚱鏁�
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
     
        %  update speed
        function [U,F,A,Fa,Fb,Fc] = speed1(obj,argu1,t,e1,e2)
            t_r = int16(10*t);
            t_r_old=int16(t_r - 1);
            
            Fa = zeros(2,1);  % vector 
            Fb = zeros(2,1);
            Fc = zeros(2,1);
            F = zeros(2,1);
            A = zeros(2,1);
            U = zeros(2,1);
            
            %获取当前坐标
            current_x = obj.Profile(1,t_r);
            current_y = obj.Profile(2,t_r);

            %计算指向终点的单位向量
            dx = obj.destination_x - current_x;
            dy = obj.destination_y - current_y;
            distToDest = sqrt(dx^2 + dy^2);
            if distToDest < 1e-6
                % 避免距离太小除零
                Ex = 0; 
                Ey = 0;
            else
                Ex = dx / distToDest;
                Ey = dy / distToDest;
            end

            %计算期望速度
            desiredSpeedMag = sqrt(obj.Va_x^2 + obj.Va_y^2); 
            
            %然后期望速度向量 = desiredSpeedMag * (Ex, Ey)
            Va_x_now = desiredSpeedMag * Ex;
            Va_y_now = desiredSpeedMag * Ey;
            
            %更新后的自驱力（考虑竖直方向上的速度分量）
            Fa(:,1) = obj.coeffFa(1,t_r) * [obj.m * (Va_x_now - obj.U(1,t_r)) / obj.Ta; obj.m * (Va_y_now - obj.U(2,t_r)) / obj.Ta];

            %旧版本的自驱力
            %Fa(:,1) = [obj.m * (obj.Va_x - obj.U(1,t_r))/obj.Ta; obj.m*(obj.Va_y - obj.U(2,t_r))/obj.Ta];
            
            %道路使用者的作用力
            Fb(:,1) = 0;

            for i = 1:length(argu1)
                if argu1{1,i}.Profile(1,t_r)<e2 && argu1{1,i}.Profile(1,t_r)>e1 && argu1{1,i}.Profile(2,t_r)~=0 && argu1{1,i}.Profile(1,t_r)>obj.Profile(1,t_r)
                    R = sqrt((obj.Profile(1,t_r) - argu1{1,i}.Profile(1,t_r))^2 + (obj.Profile(2,t_r)-argu1{1,i}.Profile(2,t_r))^2);
                    a = abs(argu1{1,i}.Profile(2,t_r) - obj.Profile(2,t_r));
                    b = abs(argu1{1,i}.Profile(1,t_r) - obj.Profile(1,t_r));
                    tan = a/b;
                    if R>0 && R<obj.Ba1 && tan < 5000 
                        fb = obj.Aa1 * exp((obj.r + argu1{1,i}.r - R)/obj.Ba1);
                        Fb(:,1) = obj.coeffFb(1,t_r) * [(obj.Profile(1,t_r) - argu1{1,i}.Profile(1,t_r))/R * fb + Fb(1,1); (obj.Profile(2,t_r) - argu1{1,i}.Profile(2,t_r))/R * fb + Fb(2,1)];
                    end
                end
            end
            
            %道路边界的作用力 
            Fc(:,1) = obj.coeffFc(1,t_r) * [0;obj.Ac * exp((obj.r - abs(obj.Profile(2,t_r) - (20-0.5)))/obj.Bo) - obj.Ac * exp((obj.r-abs(obj.Profile(2,t_r) - (22+0.5)))/obj.Bo)];
            
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
            U(1,1) = obj.U(1,t_r) + A(1,1) * obj.q;
            %if obj.U(1,t_r) + A(1,1) * obj.q >= 0
             %   U(1,1) = obj.U(1,t_r) + A(1,1) * obj.q;
            %else
              %  U(1,1) = 0;
            %end
        end

        %  update position 
        function Profile = position1(oldspeed_x,oldspeed_y,obj,t)
            t_r=int16(10*t);
            Profile = zeros(2,1);
            %if obj.Profile(2,t_r)+(oldspeed_y+obj.U(2,t_r))/2*obj.q>19.5 && obj.Profile(2,t_r)+(oldspeed_y+obj.U(2,t_r))/2*obj.q<22.5
                Profile = [obj.Profile(1,t_r)+(oldspeed_x+obj.U(1,t_r))/2*obj.q; obj.Profile(2,t_r)+(oldspeed_y+obj.U(2,t_r))/2*obj.q];
        end
        
    end
    
end
