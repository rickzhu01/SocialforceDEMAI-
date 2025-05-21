function [gradients,loss] = mseLoss(net,state,action,target)
    % 前向
    actionPred = forward(net,state);
    % MSE
    loss = mean((actionPred - target).^2,"all");
    % 反向
    gradients = dlgradient(loss,net.Learnables);
end
