function t = timestart1(lambda1)  %行人到达时间间隔
    % 设置参数lambda1
    
    % 生成随机数
    t = exprnd(1/lambda1, 1);  %生成一个服从负指数分布的随机数
end

