%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%Authors: Elif Ayvali (eayvali@gmail.com) / Hadi Salman (hadicsalman@gmail.com)
%%Biorobotics lab, The Robotics Institute, Carnegie Mellon University
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%
clc;close all;clear;
%% Initialization

opt=[];
%%%%%%%%%%%%%%%%%%%%%%  MOST IMPORTANT  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
opt.algorithm = 'KL'; % 'KL --> KL_STOEC in the paper' - 'ergodic --> ergodic_STOEC in the paper'
opt.sensorMode = 'smallFootPrint'; %choose between 'smallFootPrint' and 'largeFootPrint' if using KL
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

[opt] = initialize_gen_traj_CE(opt);
addnoise=0;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%obstacle map
img = im2double(rgb2gray(imread('obstacleMap.png')));
img = imresize(img,[opt.ng(1),opt.ng(2)]);
% figure(5);
img = flip(img);
opt.image = img;
% imshow(img);

img2 = im2double((imread('infoMapwithObstacles.png')));
img2 = imresize(img2,[opt.ng(1),opt.ng(2)]);
% figure(6);
opt.mapWithObstacles = img2;
% imshow(img2);
%%%%%%%%%%%%%%%%%%%%%%%%%%


%Generate Information Map
[X,Y,utility] = GenerateUtilityMap(opt, addnoise);
utility=reshape(utility,size(X));
Z=zeros(size(X)); %2D problem

utility = utility.*(~img);%overlay obstacles
%%
%%%%%%%%%%%%%%%%%%%% initialize CE planner options%%%%%%%%%%%%%%%%%%%
opt.X = X;
opt.Y = Y;
opt.Z = Z;
opt.xss=[X(:),Y(:),Z(:)];%Domain:3D grid
opt.utility=utility./sum(sum(utility));
opt.nagents = 3;%%just make sure to inittialize agents manually according to the number of agents you specify
opt.agent(1).xi = [[120;30;0];90*pi/180];%initial position:[x,y,z,alpha]
opt.agent(2).xi = [[130;130;0];270*pi/180];%initial position:[x,y,z,alpha]
opt.agent(3).xi = [[30;120;0];270*pi/180];%initial position:[x,y,z,alpha]

opt.agent(1).xps = opt.agent(1).xi; %accumulates trajectory points
opt.agent(2).xps = opt.agent(2).xi;
opt.agent(3).xps = opt.agent(3).xi;
opt. colors = {'m','k','w'};% Thecolor of the trajectory of each agent i.e. same size as number of agents
opt.kdOBJ = KDTreeSearcher(opt.xss(:,1:2));%to eval traj cost by closest point

%%%%%%%%%%%%%%%%%%%%% initialize ergodicity %%%%%%%%%%%%%%%%%%%%%%%%%
%Calculate Fourier Coeff of utility for ergodic coverage
opt.erg.mu=opt.utility;
Nk = 10;% Number of wave-numbers to be used
opt.erg.Nkx = Nk;
opt.erg.Nky = Nk;
[opt.erg.muk] = GetFourierCoeff(opt,X,Y);
opt.erg.Ck = zeros(Nk,Nk);
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
BhattDistance = zeros(opt.stages,1);
save_traj_stat = zeros(150*150,opt.stages);
euclidean_dist = zeros(opt.stages,1);
for k=1:opt.stages
    opt.currentStage = k
    for iagent = 1:opt.nagents
    opt.iagent = iagent;
    [opt,xs] = gen_traj_CE(opt);

    set(opt.agent(iagent).trajFigOPTIMAL,'XData', [opt.agent(iagent).xps(1,:),xs(1,:)]);
    set(opt.agent(iagent).trajFigOPTIMAL,'YData', [opt.agent(iagent).xps(2,:),xs(2,:)]);
    set(opt.agent(iagent).trajFigOPTIMAL,'ZData', [opt.agent(iagent).xps(3,:),xs(3,:)]);
    
    xf = xs(:,end); %end of the horizon
    opt.agent(iagent).xi = xf;
    opt.agent(iagent).xps = [opt.agent(iagent).xps,xs(:,2:end)];%accumulate trajectory (xs starts from two in order to avoid double counting endpoints!)
 
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   
    %%%%%%%%%%%%%%%%  Calculate Ck  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  
    
    if (strcmp(opt.algorithm,'ergodic'))
        opt = accumulate_CK(opt, xs);% updates CK of the whole trajectory (for efficient calculation)
    end
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%       Trajectory time-average statistics      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%         
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    %%%%%%%%%%%%%%%%%%   large footPrint sensor %%%%%%%%%%%%%%%   
    if (strcmp(opt.sensorMode,'largeFootPrint') && strcmp(opt.algorithm,'KL') )  
        [~, ~, fmu, fs2]= gp(opt.gp_para,opt.gp_model.inf, opt.gp_model.mean, opt.gp_model.cov, opt.gp_model.lik, xs(1:opt.dim,50:50:end)', ones(length(xs(1,50:50:end)),1), opt.xss);
        opt.traj_stat = opt.traj_stat + reshape(fmu,Lx,Ly);
        set(opt.htraj,'CData',[opt.traj_stat]);
    end
    
    %%%%%%%%%%%%%%%%%%%   identity sensor       %%%%%%%%%%%%%%%%
    if (  ((strcmp(opt.algorithm,'KL')) && strcmp(opt.sensorMode,'smallFootPrint')) ||  strcmp(opt.algorithm,'ergodic'))     
        ind = knnsearch(opt.kdOBJ, xs(1:2,:)');
        opt.traj_stat = opt.traj_stat + reshape(accumarray(ind,1,size(opt.traj_stat(:))),Lx,Ly);
        set(opt.htraj,'CData',[opt.traj_stat]);
    end
    
    end
    traj_stat_normalized = opt.traj_stat./sum(sum(opt.traj_stat));
    save_traj_stat(:,k) = traj_stat_normalized(:);
    euclidean_dist(k) = sum(sum((traj_stat_normalized - opt.utility).^2));
    BhattDistance(k) = evaluateBhattacharyyaDist( traj_stat_normalized, opt );

end
