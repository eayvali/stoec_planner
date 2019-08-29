%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%Authors: Elif Ayvali (eayvali@gmail.com) / Hadi Salman (hadicsalman@gmail.com)
%%Biorobotics lab, The Robotics Institute, Carnegie Mellon University
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%
clc;close all;clear;
%% Initialization

opt=[];
%%%%%%%%%%%%%%%%%%%%%%  MOST IMPORTANT  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
opt.sensorMode = 'largeFootPrint'; %choose between 'smallFootPrint' and 'largeFootPrint'
opt.algorithm = 'KL'; % 'KL--> KL_path_planning'
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

[opt] = initialize_gen_traj_CE(opt);
addnoise=0;
%Generate Information Metric
[X,Y,utility] = GenerateUtilityMap(opt, addnoise);
utility=reshape(utility,size(X));
Z=zeros(size(X)); %2D problem

%%
%%%%%%%%%%%%%%%%%%%% initialize CE planner options%%%%%%%%%%%%%%%%%%%
opt.xss=[X(:),Y(:),Z(:)];%Domain:3D grid
opt.utility=utility./sum(sum(utility));
opt.nagents = 1;%%just make sure to inittialize agents manually according to the number of agents you specify
opt.agent(1).xi = [[30;30;0];90*pi/180];%initial position:[x,y,z,alpha]

opt.agent(1).xps = opt.agent(1).xi; %accumulates trajectory points

opt. colors = ['m','g', 'r'];% Thecolor of the trajectory of each agent i.e. same size as number of agents
opt.kdOBJ = KDTreeSearcher(opt.xss(:,1:2));%to eval traj cost by closest point

%--------------------------------------------------------%
%%%%%%%%%%%%%%%%initialize GP%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%GP parameters
opt.gp_model = struct('inf',@infExact, 'mean', @meanZero, 'cov', @covSEiso, 'lik', @likGauss);
sn = 0.01; ell = 7 ; sf = sqrt(1);Ncg=30;
opt.gp_para.lik = log(sn); opt.gp_para.cov = log([ell; sf]);
fmu=zeros(size(opt.xss(:,1)));
fs2=zeros(size(opt.xss(:,1)));

Lx = opt.ng(1);
Ly = opt.ng(2);
opt.traj_stat=zeros(Lx,Ly);

%%
%%%%%%%%%%%%%%%%%%%%%%%   plot initail distributions and trajecrtories  %%%%%%%%%%%%%%%%%%%%%
opt = plot_initial(opt);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Cross entropy optimization
% pause(4);
tic
for k=1:opt.stages
    opt.currentStage = k;
    for iagent = 1:opt.nagents
    opt.iagent = iagent;
    [opt,xs] = gen_traj_CE(opt);

    set(opt.agent(iagent).trajFigOPTIMAL,'XData', [opt.agent(iagent).xps(1,:),xs(1,:)]);
    set(opt.agent(iagent).trajFigOPTIMAL,'YData', [opt.agent(iagent).xps(2,:),xs(2,:)]);
    set(opt.agent(iagent).trajFigOPTIMAL,'ZData', [opt.agent(iagent).xps(3,:),xs(3,:)]);
    
    xf = xs(:,end); %end of the horizon
    opt.agent(iagent).xi = xf;
    opt.agent(iagent).xps = [opt.agent(iagent).xps,xs(:,2:end)];%accumulate trajectory (xs starts from two in order to avoid double counting endpoints!)
 
       
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%       Trajectory time-average statistics      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%         
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    %%%%%%%%%%%%%%%%%%   large footPrint sensor %%%%%%%%%%%%%%%   
    if (strcmp(opt.sensorMode,'largeFootPrint') && strcmp(opt.algorithm,'KL') )  
        [~, ~, fmu, fs2]= gp(opt.gp_para,opt.gp_model.inf, opt.gp_model.mean, opt.gp_model.cov, opt.gp_model.lik, xs(1:opt.dim,50:50:end)', ones(length(xs(1,50:50:end)),1), opt.xss);
        opt.traj_stat = opt.traj_stat + reshape(fmu,Lx,Ly);
        set(opt.htraj,'CData',[opt.traj_stat(:) opt.traj_stat(:)]);
    end
    
    %%%%%%%%%%%%%%%%%%%   identity sensor       %%%%%%%%%%%%%%%%
    if (  ((strcmp(opt.algorithm,'KL')) && strcmp(opt.sensorMode,'smallFootPrint')) ||  strcmp(opt.algorithm,'ergodic'))     
        ind = knnsearch(opt.kdOBJ, xs(1:2,:)');
        opt.traj_stat = opt.traj_stat + reshape(accumarray(ind,1,size(opt.traj_stat(:))),Lx,Ly);
        traj_stat_normalized = opt.traj_stat./sum(sum(opt.traj_stat));
        set(opt.htraj,'CData',[traj_stat_normalized(:) traj_stat_normalized(:)]);
    end
    
    end
end
