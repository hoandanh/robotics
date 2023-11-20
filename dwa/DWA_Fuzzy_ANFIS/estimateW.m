function weights = estimateW(toObs,toGoal,heading,config)
    alpha = evalfis(config.alpha,[toObs;toGoal;heading]);
    beta = evalfis(config.beta,[toObs;toGoal;heading]);
    gamma = evalfis(config.gamma,[toObs;toGoal;heading]);
    
    weights = [alpha,beta,gamma];
end