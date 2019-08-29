%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%Authors: Elif Ayvali (eayvali@gmail.com) / Hadi Salman (hadicsalman@gmail.com)
%%Biorobotics lab, The Robotics Institute, Carnegie Mellon University
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%
clear all;clc;close all
%%
[pose, opt] = initialization();
addnoise=0;
[X,Y,informationMap] = GenerateUtilityMap(opt,addnoise);
Z = zeros(size(X));

% %%%overwrite utility
% img = im2double(rgb2gray(imread('apple.jpg')));
% img = imresize(img,[150,150]);
% img =  imbinarize(img);
% %imshow(img);
% informationMap = flip(img);
% informationMap = informationMap./sum(sum(informationMap));
% %%%


opt.erg.mu=reshape(informationMap,size(X));
%%%%%%%%%%%% not necessary anymore...I transposed the muk matrix inside
%GetFourierCoeff
    %         opt.erg.mu=flipud(opt.erg.mu);
    %         opt.erg.mu=imrotate(opt.erg.mu,-90);
    %         imshow(opt.erg.mu)
%%%%%%%%%%%%%
opt.erg.mu = opt.erg.mu/sum(sum(opt.erg.mu));% normalize iformation distribution
[opt.erg.muk, opt.erg.HK] = GetFourierCoeff(opt,X,Y);
opt.kdOBJ = KDTreeSearcher([X(:),Y(:)]);%to eval traj cost by closest point

%% Plot utility 
figure(1);set(gcf,'color','w'); hold on
surface(X,Y,Z,reshape(informationMap,size(X)), 'FaceColor','interp', 'EdgeColor','interp','Marker','.');
axis tight
axis equal
% colorbar;


%% run simulation
Nsteps = opt.sim.Nsteps;
dt = opt.sim.dt;

% Initializing Fourier coefficients of coverage distribution
Ck = zeros(opt.erg.Nkx, opt.erg.Nky);
figure(1);
colors= {'k','g','m'};
Ergodicity_Metric = zeros(Nsteps,1);
BhattDistance = zeros(Nsteps,1);

for iagent = 1:opt.nagents
    scatter(pose.x(iagent),pose.y(iagent),'filled','MarkerFaceColor',[0,0,0],'MarkerEdgeColor',[1,1,1]);%plot posistion of agents initially
%     text(pose.x,pose.y,'\leftarrow start','Color','r')
    h(iagent) = plot(pose.x(iagent),pose.y(iagent), 'Marker', 'o', 'MarkerSize', 2,'Color', [0,0,0]);    
end

traj = zeros(Nsteps, opt.nagents, 3);%%(iteration * agent * [x,y])        
% saveas(gcf,'initial_SMC.fig')

%%
% pause(5);%pause to have time to take video
tic
for it = 1:Nsteps
    if(it == Nsteps/2)
        saveas(gcf,'halfway_SMC.fig')
    end
    time = (it) * dt;
    [pose, Ck] = SMC_Update(pose, Ck, time, opt);
    traj_stat = zeros(size(opt.erg.mu(:)));
    for iagent = 1:opt.nagents
        traj(it,iagent,1)= pose.x(iagent);
        traj(it,iagent,2)= pose.y(iagent);
        set(h(iagent),'XData',traj(1:it,iagent,1),'YData',traj(1:it,iagent,2));
    
        traj_stat = traj_stat + timeAverageStatisticsDistribution( reshape(traj(:,iagent,1:2),Nsteps,2), opt); 
    end
    %%just for speed, draw every bla iterations
    if( mod(it,5) == 0)
        drawnow
    end
    
    disp(it);
    %%%%%%%%%%%%%%%    metric evaluations  %%%%%%%%%%%%%%%%%%%%
    %%%ergodicity
    ck = Ck/opt.nagents/time;
    [Ergodicity_Metric(it)] = Calculate_Ergodicity(ck, opt);
    
    %%%Bhatttacharyya distance
    BhattDistance(it) = evaluateBhattacharyyaDist( traj_stat, opt );
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
end
simulationTime = toc;
saveas(gcf,'end SMC.fig')
%plot positions of agents at the end of the trajectory
% for iagent = 1:opt.nagents
%         scatter(pose.x(iagent),pose.y(iagent),colors(iagent),'fill');
% %         text(pose.x,pose.y,'\leftarrow start','Color','r')
% end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                          PLOTS                             %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% %% plot metric of ergodicity as a function of time
% time=dt:dt:dt*Nsteps;
% figure;loglog(time(1:end),Ergodicity_Metric(1:end))
% % axis([0.001 5 0.0001,1])
% xlabel('Time (sec)');
% ylabel('Coverage Metric, \phi(t)');
% title('metric of ergodicity as a function of time')
% 
%% plot "Bhatttacharyya distance" (between information distribution and time averaged statistics of trajectories) as a function of time
time=dt:dt:dt*Nsteps;
figure;plot(time(1:end),BhattDistance(1:end))
% axis([0.001 5 0.0001,1])
xlabel('Time (sec)');
ylabel(',Bhatttacharyya distance (BhDist(t))');
title('Bhatttacharyya distance as a function of time')



% 
% %%
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% %%%%%%%%%%%%%%%%%%%%%%%%%%%         Trajectory plots for IROS paper    %%%%%%%%%%%%%%%%%%%%%%%  
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% clear; close all;
% load SMC1.mat
% 
% Nsteps = opt.sim.Nsteps;
% 
% 
% %t = 0
% figure(1);set(gcf,'color','w'); hold on
% surface(X,Y,Z,reshape(informationMap,size(X)), 'FaceColor','interp', 'EdgeColor','interp','Marker','.');
% axis tight
% axis equal
% colorbar;
% scatter(140,20,'r','fill')
% title('t = 0 sec')
% 
% %t = 500 sec
% figure(2);set(gcf,'color','w'); hold on
% surface(X,Y,Z,reshape(informationMap,size(X)), 'FaceColor','interp', 'EdgeColor','interp','Marker','.');
% axis tight
% axis equal
% colorbar;
% scatter(140,20,'r','fill')
% plot3(traj(1:Nsteps/2,:,1),traj(1:Nsteps/2,:,2),traj(1:Nsteps/2,:,3),'r-','LineWidth', 2, 'MarkerSize', 5);
% title('t = 500 sec')
% 
% %t = 1000 sec
% figure(3);set(gcf,'color','w'); hold on
% surface(X,Y,Z,reshape(informationMap,size(X)), 'FaceColor','interp', 'EdgeColor','interp','Marker','.');
% axis tight
% axis equal
% colorbar;
% scatter(140,20,'r','fill')
% plot3(traj(1:Nsteps,:,1),traj(1:Nsteps,:,2),traj(1:Nsteps,:,3),'r-','LineWidth', 2, 'MarkerSize', 5);
% title('t = 1000 sec')
% 
% 
% % plot "Bhatttacharyya distance" (between information distribution and time averaged statistics of trajectories) as a function of time
% figure(4)
% dt = opt.sim.dt;
% time=dt:dt:dt*Nsteps;
% plot(time(1:Nsteps),BhattDistance(1:Nsteps));
% xlabel('Time (sec)');
% ylabel(',Bhatttacharyya distance (BhDist(t))');
% title('Bhatttacharyya distance as a function of time')
