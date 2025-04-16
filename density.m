function density = computeLocalDensity(observer, P_collection, t_r)
%仅计算前方140都视角范围内的行人
%仅计算距离2米内的行人
%密度为符合要求行人与观察者之间距离的倒数之和。

    %获取观察者的当前位置与速度
    obsPos = observer.Profile(:, t_r);   % [x; y]
    obsVel = observer.U(:, t_r);         % [vx; vy]
    
    % 若速度太小默认朝向x轴正方向
    if norm(obsVel) < 1e-6
        obsVel = [1; 0];
    end

    %得到观察者朝向 单位向量
    obsDir = obsVel / norm(obsVel);  % 方向单位向量
    
    densitySum = 0;
    numPeople = length(P_collection);  % 所有行人数
    
    for i = 1 : numPeople
        
        %跳过自己
        if P_collection{i}.N == observer.N
            continue; 
        end
        
        %计算距离
        otherPos = P_collection{i}.Profile(:, t_r);
        delta = otherPos - obsPos;         % 相对向量
        dist  = norm(delta);              % 距离
        
        % 若距离大于2 米，不计入
        if dist > 2
            continue;
        end
        
        % 距离过小1/0会趋向无穷
        if dist < 1e-6
            continue;  
        end
        
        %计算视角（delta 与 obsDir 的夹角）
        % angle = arccos( (obsDir · delta方向单位向量) ) ，再转换为度数
        deltaDir = delta / dist;  % 单位向量
        dotVal   = dot(obsDir, deltaDir); %点乘
        % 由于数值误差，dotVal 可能略超[-1,1], 用min/max保护
        dotVal = max(min(dotVal, 1), -1);
        
        angleRad = acos(dotVal);      % 弧度
        angleDeg = angleRad * 180/pi; % 转为度数
        
        % 判断
        if angleDeg <= 70
            % density 是r倒数之和
            densitySum = densitySum + (1 / dist);
        end
    end

    density = densitySum;
end
