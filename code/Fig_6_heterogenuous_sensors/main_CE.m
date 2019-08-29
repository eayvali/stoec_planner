%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%Authors: Elif Ayvali (eayvali@gmail.com) / Hadi Salman (hadicsalman@gmail.com)
%%Biorobotics lab, The Robotics Institute, Carnegie Mellon University
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%
% clc;close all;clear;
%% Initialization

opt=[];

opt.algorithm = 'KL'; % 'KL'

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

%Generate Information Metric
[X,Y,utility] = GenerateUtilityMap(opt, addnoise);
utility=reshape(utility,size(X));
Z=zeros(size(X)); %2D problem

utility = utility.*(~img);%overlay obstacles
%% beam sensor image processing: Creat lookup table
img = im2double(rgb2gray(imread('beam.png')));
img = imresize(img,[opt.ng(1),opt.ng(2)]);
% figure(5);
img = flip(img);
% opt.image = img;
bias_row = +6;
bias_col = +3;
imgTrans = imtranslate(img,[bias_row,bias_col]);
tic
for theta=1:10:360
    imgRot = imrotate(imgTrans,theta);
    
    %%%%%%%%%%%%%%%%% warper  %%%%%%%%%%%%%%%%%
    [p3, p4] = size(imgRot);
    q1 = opt.ng(1);
    q2 = opt.ng(2);
    i3_start = floor((p3-q1)/2)+1; % or round instead of floor; using neither gives warning
    i3_stop = i3_start + q1-1;
    i4_start = floor((p4-q2)/2)+1;
    i4_stop = i4_start + q2-1;
    II = imgRot(i3_start:i3_stop, i4_start:i4_stop);
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    II = imtranslate(II,[0,0]);
%     imshow(II);
end
toc


%%
%%%%%%%%%%%%%%%%%%%% initialize CE planner options%%%%%%%%%%%%%%%%%%%
opt.X = X;
opt.Y = Y;
opt.Z = Z;
opt.xss=[X(:),Y(:),Z(:)];%Domain:3D grid
opt.utility=utility./sum(sum(utility));
opt.nagents = 3;%%just make sure to inittialize agents manually according to the number of agents you specify
opt.agent(1).xi = [[120;30;0];90*pi/180];%initial position:[x,y,z,alpha]
opt.agent(2).xi = [[30;30;0];90*pi/180];%initial position:[x,y,z,alpha]
opt.agent(3).xi = [[30;120;0];270*pi/180];%initial position:[x,y,z,alpha]

opt.agent(2).sensorMode = 'largeFootPrint'; %choose between  'smallFootPrint' and 'largeFootPrint'
opt.agent(3).sensorMode = 'largeFootPrint'; %choose between  'smallFootPrint' and 'largeFootPrint'
opt.agent(1).sensorMode = 'smallFootPrint'; %choose between  'smallFootPrint' and 'largeFootPrint'

opt.agent(1).xps = opt.agent(1).xi; %accumulates trajectory points
opt.agent(2).xps = opt.agent(2).xi;
opt.agent(3).xps = opt.agent(3).xi;
opt. colors = {'m','k','w'};% Thecolor of the trajectory of each agent i.e. same size as number of agents
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
save_traj_stat = zeros(150*150,opt.stages);
euclidean_dist = zeros(opt.stages,1);
BhattDistance = zeros(opt.stages,1);
for k=1:opt.stages
    opt.currentStage = k
    for iagent = 1:opt.nagents
       
        opt.sensorMode = opt.agent(iagent).sensorMode;
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
            opt.traj_stat = opt.traj_stat + 4*reshape(fmu,Lx,Ly);
            set(opt.htraj,'CData',[opt.traj_stat]);
    %         traj_stat_normalized = opt.traj_stat./sum(sum(opt.traj_stat));
    %         set(opt.htraj,'CData',[traj_stat_normalized(:) traj_stat_normalized(:)]);
        end

        %%%%%%%%%%%%%%%%%%%   identity sensor       %%%%%%%%%%%%%%%%
        if (  ((strcmp(opt.algorithm,'KL')) && strcmp(opt.sensorMode,'smallFootPrint')) ||  strcmp(opt.algorithm,'ergodic'))     
            ind = knnsearch(opt.kdOBJ, xs(1:2,:)'); 
            opt.traj_stat = opt.traj_stat + reshape(accumarray(ind,1,size(opt.traj_stat(:))),Lx,Ly);
            traj_stat_normalized = opt.traj_stat;%./sum(sum(opt.traj_stat));
            set(opt.htraj,'CData',[traj_stat_normalized]);
        end
    end
    traj_stat_normalized = opt.traj_stat./sum(sum(opt.traj_stat));
    save_traj_stat(:,k) = traj_stat_normalized(:);
    euclidean_dist(k) = sum(sum((traj_stat_normalized - opt.utility).^2));
    BhattDistance(k) = evaluateBhattacharyyaDist( traj_stat_normalized, opt );
end

