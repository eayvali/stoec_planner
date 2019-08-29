function [f] = traj_cost(z, opt)

xstemp = traj(z, opt);

%Cost functions
%% 1)Ergodic coverage
if (strcmp(opt.algorithm,'ergodic'))
    f = EvaluateErgodicity(xstemp(1:opt.dim,:),opt); %current stage
end
%% 2)KL divergence coverage
if (strcmp(opt.algorithm,'KL'))
    [f,~] = EvaluateKLcoverage(xstemp(1:opt.dim,:),opt);
end

%%

% check for bounds
for i=1:2
    if sum(find(xstemp(i,:) < opt.xlb(i))) || sum(find(xstemp(i,:) > opt.xub(i)))
        f = 1000;
        return
    end
end
end
