function [opt, xs] = gen_traj_CE(opt)
iagent = opt.iagent;
opt.ce.C = .5*opt.ce.C + .5*opt.ce.C0;
z = opt.ce.z0;
tic
for i=1:opt.iters
    opt.z = z;
    opt.ceFlag=1; %plot trajectories inside cem
    [z, ~, ~, C] = cem(@traj_cost, z, opt.ce, opt);
    opt.ce.C = C;
    xs = traj(z, opt);
    
    set(opt.agent(iagent).trajFig,'XData', [opt.agent(iagent).xps(1,:),xs(1,:)]);
    set(opt.agent(iagent).trajFig,'YData', [opt.agent(iagent).xps(2,:),xs(2,:)]);
    if(opt.dim == 3)
       set(opt.agent(iagent).trajFig,'ZData', [opt.agent(iagent).xps(3,:),xs(3,:)]);         
    end
    set(opt.agent(iagent).trajFig,'Color', opt.colors{iagent});
    drawnow
end
toc
end