function [f] = traj_cost(z, opt)

xstemp = traj(z, opt);

%Cost functions
%% 1)KL divergence coverage
if (strcmp(opt.algorithm,'KL'))
    [f, opt] = EvaluateKLcoverage(xstemp(:,:),opt);
end

%%
%penalize if crosses obstacle
if opt.traj_stat_temp(:)'*opt.image(:) > 0
    f = f+ 1000;
end


noFlyZone = 0;
%penalty for visiting prohibited region (e.g.,location of hostiles)
global r x1 x2 y1 y2
if noFlyZone==1
dist1 = sqrt((x1 - xstemp(1,:)).^2   +   (y1 - xstemp(2,:)).^2); % distance calc.
dist2 = sqrt((x2 - xstemp(1,:)).^2   +   (y2 - xstemp(2,:)).^2); % distance calc.
in1 = find(dist1<r);
in2 = find(dist2<r);
f = f + 100*(length(in1)+length(in2));
% display(f);
end

% check for bounds
for i=1:2
    if sum(find(xstemp(i,:) < opt.xlb(i))) || sum(find(xstemp(i,:) > opt.xub(i)))
        f = 1000;
        return
    end
end

end
