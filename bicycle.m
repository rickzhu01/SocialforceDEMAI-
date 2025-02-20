classdef bicycle  %���г���
    %bikes are the object which run on roads

    %   �˴���ʾ��ϸ˵��
    
    properties      %������
        t0;         % entering time 
        Name;
        N;          % Serial number of persons
        Profile;    % distance time profile
        Fa          % ������
        Fb          % ��·ʹ����������
        Fc          % �߽�������
        F;          % ����
        A;          % ���ٶ�
        U;          % �ٶ�
        q = 0.1;      % delta t
        Va_x = 0;     %�����ٶ�
        Va_y;     %�����ٶ� 
        Ta = 1;       %���巴Ӧʱ��
        Aa1 = 300;     %���г��Ը����������ǿ��ϵ��
        Aa2 = 500;     %���˶Ը����������ǿ��ϵ��
        Ba1 = 4.0;       %��������������÷�Χ
        Ba2 = 5.2;
        r = 0.9;      %����뾶
        Ao = 150;     %�ϰ����ų���ǿ��ϵ��
        Bo = 1;       %������ϰ�����������÷�Χ
        m ;       %�����г��˵�����
        V = 5;        %����ٶ�
        
        I = 15.75;             %���Ծ� ��/4*a*b^3
        ko = 1;
        kd = 500;
        kl = 0;
        alpha = 3;
        
     
    end
      
  %  properties (Dependent)
         
  %  end
           

    methods
        function AM = bicycle(t0,N,Profile,Fa,Fb,Fc,F,U,A,q,Ta,Aa1,Aa2,Ba,Bo,r,Ao,m,V,I,ko,kd,kl,alpha)%Construction functionr���캯��
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
     
          %�����ٶ�  argu1=���г����ϣ�argu2=���˼���
        function [U,F,A,Fa,Fb,Fc] = speed2(obj,argu1,argu2,t,e1,e2)
            t_r = int16(10*t);
            t_r_old = int16(t_r - 1);
            
            Fa = zeros(2,1);
            Fb = zeros(2,1);
            Fc = zeros(2,1);
            F = zeros(2,1);
            A = zeros(2,1);
            U = zeros(2,1);
            
            %������
            Fa(:,1) = [obj.m*(obj.Va_x - obj.U(1,t_r))/obj.Ta; obj.m*(obj.Va_y - obj.U(2,t_r))/obj.Ta];
                        
            %��·ʹ���ߵ�������
            Fb(:,1) = 0;

            for i = 1 : length(argu1)
                if argu1{1,i}.Profile(1,t_r)<e2 && argu1{1,i}.Profile(1,t_r)>e1 && obj.Profile(2,t_r)<argu1{1,i}.Profile(2,t_r)
                    R = sqrt((obj.Profile(1,t_r) - argu1{1,i}.Profile(1,t_r))^2 + (obj.Profile(2,t_r) - argu1{1,i}.Profile(2,t_r))^2);
                    a = abs(obj.Profile(2,t_r) - argu1{1,i}.Profile(2,t_r));%�������
                    b = abs(obj.Profile(1,t_r) - argu1{1,i}.Profile(1,t_r));%�������
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
                    a = abs(obj.Profile(2,t_r) - argu2{1,i}.Profile(2,t_r));%�������
                    b = abs(obj.Profile(1,t_r) - argu2{1,i}.Profile(1,t_r));%�������
                    tan = b/a;
                    if R>0 && R<obj.Ba2 && tan<1.732 
                        fb = obj.Aa2 * exp((obj.r + argu2{1,j}.r - R)/obj.Ba2);
                        Fb(:,1) = [(obj.Profile(1,t_r) - argu2{1,j}.Profile(1,t_r))/R * fb+Fb(1,1);(obj.Profile(2,t_r) - argu2{1,j}.Profile(2,t_r))/R * fb + Fb(2,1)];
                    end
                end
            end
            
            %��·�߽��������
            Fc(:,1) = [obj.Ao * exp((obj.r - abs(obj.Profile(1,t_r) - e1))/obj.Bo) - obj.Ao * exp((obj.r - abs(obj.Profile(1,t_r) - e2))/obj.Bo); 0];
            
            %�������ص�����

            ktheta = obj.I * obj.kl * Fa(1,1);
            komega = obj.I * (1 + obj.alpha) * sqrt(obj.kl * abs(Fa(1,1))/obj.alpha);
            Fu1 = obj.ko * (Fa(1,1) + Fb(1,1) + Fc(1,1)) - obj.kd * obj.U(1,t_r);        %������
            if (t_r>1)
                theta_i = atan2(obj.U(1,t_r),obj.U(2,t_r));                 %�����1
                theta_i_old = atan2(obj.U(1,t_r_old),obj.U(2,t_r_old));     %�����2
            else
                theta_i = atan(obj.U(1,t_r) / obj.U(2,t_r));
                theta_i_old = 0;
            end

            omega_i = (theta_i - theta_i_old) / obj.q;                      %���ٶ�
            u_theta = -ktheta * (theta_i - 0) - komega * omega_i;            %����
            Fu2 = -(u_theta / obj.I) * cos(theta_i);                      %����תΪ����������ʽ
            
            %�������
            F(:,1) = [Fu1 + Fu2; Fa(2,1) + Fb(2,1) + Fc(2,1)];
            
            %������ٶ�
            A(:,1) = [F(1,1)/obj.m; F(2,1)/obj.m];
           
            %�����ٶ�
            U(1,1) = obj.U(1,t_r) + A(1,1) * obj.q;
            if obj.U(2,t_r) + A(2,1) * obj.q >= 0
                U(2,1) = obj.U(2,t_r) + A(2,1) * obj.q;
            else
                U(2,1) = 0;
            end
        end

        %����λ��
        function Profile = position2(oldspeed_x,oldspeed_y,obj,t)
            t_r = int16(10*t);
            Profile = zeros(2,1);
            Profile = [obj.Profile(1,t_r) + ((oldspeed_x + obj.U(1,t_r))/2) * obj.q; obj.Profile(2,t_r) + ((oldspeed_y + obj.U(2,t_r))/2) * obj.q];
        end
       
    end
    
end
