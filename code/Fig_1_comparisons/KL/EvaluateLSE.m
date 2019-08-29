function [ LSE ] = EvaluateLSE( traj,opt )

ind = knnsearch(opt.kdOBJ, traj(1:2,:)');
traj_stat = accumarray(ind,1,size(opt.traj_stat(:)));
traj_stat=traj_stat./sum(sum(traj_stat));%normalizing

%Computationally expensive, but intuitive:
DomainBounds.xmin = opt.xlb(1);
DomainBounds.xmax = opt.xub(1);
DomainBounds.ymin = opt.xlb(2);
DomainBounds.ymax = opt.xub(2);
Lx = DomainBounds.xmax - DomainBounds.xmin;
Ly = DomainBounds.ymax - DomainBounds.ymin;
%traj:xs(1:opt.dim,:)
traj=traj';

% % % %this doesn't incorporate crossing same point twice->needs to be modified
% % % %take the sensor width into account determined by the kernel
% % % [~, ~, fmu, ~]= gp(opt.gp_para,opt.gp_model.inf, opt.gp_model.mean, opt.gp_model.cov, opt.gp_model.lik, traj, ones(size(traj(:,1))),opt.xss);
% % % fmu=reshape(fmu,Lx,Ly);
% % % traj_stat=fmu./sum(sum(fmu));

LSE = sqrt(sum(opt.utility(:).^2 - traj_stat(:).^2));
%make sure they are both pdf
% KL_dist=sum(opt.utility(:).*log(opt.utility(:)+eps)-opt.utility(:).*log(traj_stat(:)+eps));

end

