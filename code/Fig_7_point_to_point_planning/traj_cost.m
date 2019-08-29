function [f] = traj_cost(z, opt)

xstemp = traj(z, opt);

xstemp_full = [opt.agent(1).xps, xstemp]; %full trajectory 

%Cost function
%% KL divergence coverage
if (strcmp(opt.algorithm,'KL'))
    xf=[30 ;140];%goal position
    [f, ~] = EvaluateKLcoverage(xstemp(1:opt.dim,:),opt)
    f = f + .1 *norm([xstemp_full(1:opt.dim-1,end);xstemp_full(4,end)]-[xf;pi/2]);  
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
