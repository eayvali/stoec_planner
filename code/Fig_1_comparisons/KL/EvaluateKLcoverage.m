function [KL_dist,opt]= EvaluateKLcoverage(traj,opt)
DomainBounds.xmin = opt.xlb(1);
DomainBounds.xmax = opt.xub(1);
DomainBounds.ymin = opt.xlb(2);
DomainBounds.ymax = opt.xub(2);
Lx = DomainBounds.xmax - DomainBounds.xmin;
Ly = DomainBounds.ymax - DomainBounds.ymin;

if (strcmp(opt.sensorMode,'smallFootPrint'))
    ind = knnsearch(opt.kdOBJ, traj(1:2,:)');%closest point on the grid
    traj_stat =  opt.traj_stat + reshape(accumarray(ind,1,size(opt.traj_stat(:))),Lx,Ly);%Bug fixed
    traj_stat = traj_stat./sum(sum(traj_stat));%make it a pdf by normalizing
end

if (strcmp(opt.sensorMode,'largeFootPrint'))
    traj=traj';
    [~, ~, fmu, ~]= gp(opt.gp_para,opt.gp_model.inf, opt.gp_model.mean, opt.gp_model.cov, opt.gp_model.lik, traj(50:50:end,:), ones(size(traj(50:50:end,1))),opt.xss);
    traj_stat = opt.traj_stat + reshape(fmu,Lx,Ly);
    traj_stat=traj_stat./sum(sum(traj_stat));%make it a pdf by normalizing
    %%%%%%%%%%%%%%
    %%plot time average statistics for each candidate trajectory
%     set(opt.htraj,'CData',[traj_stat(:) traj_stat(:)]);
    %%%%%%%%%%%%%%
end

%make sure they are both pdf
KL_dist=sum(opt.utility(:).*log(opt.utility(:)+eps)-opt.utility(:).*log(traj_stat(:)+eps));
% KL_dist=sum(traj_stat(:).*log(traj_stat(:)+eps)-traj_stat(:).*log(opt.utility(:)+eps));

%% test renyi with alpha=0.5
% alpha =.5;
% KL_dist = 1/(alpha-1)*log(sum(opt.utility(:).^alpha./traj_stat(:).^(alpha-1)));

%  set(opt.htraj,'CData',[traj_stat(:) traj_stat(:)]);
