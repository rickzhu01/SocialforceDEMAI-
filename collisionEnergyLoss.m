function [totalLoss, pairLoss] = collisionEnergyLoss(P_collect, step, k_n, c_d, dt, radius)
%DETECTCOLLISIONENERGY  计算当前时间步行人之间碰撞的能量损失

%   step 时间步骤索引
%   k_n  法向弹簧刚度
%   c_d  阻尼系数
%   dt  与 DEM 一致的积分步长
%   radius     行人半径

%   totalLoss  该帧全部能量损失之和 /J
%   pairLoss   N×3 矩阵，每行 [i , j , loss(J)]

    np = numel(P_collect);
    pairLoss   = [];
    totalLoss  = 0.0;

    for i = 1:np-1
        % 当前行人 i 的位置速度
        pi = P_collect{i}.Profile(:,step);
        vi = P_collect{i}.U(:,step);

        for j = i+1:np
            pj = P_collect{j}.Profile(:,step);
            vj = P_collect{j}.U(:,step);

            delta  = pj - pi;                 % 位移向量
            dist   = norm(delta);             % 距离
            overlap = 2*radius - dist;        % 重叠量 (>0 → 碰撞)

            if overlap > 0 && dist > eps
                n   = delta / dist;           % 单位法向
                vrel_n = (vj - vi)' * n;      % 相对速度法向分量 (标量)

                % 压缩弹簧储能
                Espring  = 0.5 * k_n * overlap^2;

                % 阻尼耗散
                Edashpot = c_d * vrel_n^2 * dt;

                Eloss = Espring + Edashpot;

                % 计算结果
                pairLoss = [pairLoss ; i , j , Eloss];
                totalLoss = totalLoss + Eloss;
            end
        end
    end
end
  