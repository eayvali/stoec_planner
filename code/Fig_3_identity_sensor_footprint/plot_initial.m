function [ opt  ] = plot_initial( opt )

%Plot utility and trajectories
figure(1); set(gcf,'color','w');
subplot(1,3,2)
set(gca,'FontSize',20);
hold on
% opt.h_utility = surface(opt.X,opt.Y,opt.Z,reshape(opt.utility,size(opt.X)), 'FaceColor','interp', 'EdgeColor','interp','Marker','.');
imagesc(flip(opt.mapWithObstacles));
axis tight
axis equal
% colorbar;
%plot regions to avoid
global r x1 x2 y1 y2
r=10;
ang=0:.01:2*pi;
x1=65;y1=40;
x2=35;y2=75;
hold on
% plot(x1+r*sin(ang),y1+r*cos(ang),'w','LineWidth',2); % r is the radius
% plot(x2+r*sin(ang),y2+r*cos(ang),'w','LineWidth',2); % r is the radius
axis equal
axis([ opt.DomainBounds.xmin opt.DomainBounds.xmax opt.DomainBounds.ymin opt.DomainBounds.ymax])
view(0,90)
% title('Red:Chosen trajectory, Blue:Two candidates #of cem iteration)')
title('Optimal trajectory')
figure(1);
subplot(1,3,1); hold on
set(gca,'FontSize',20);
% surface(opt.X,opt.Y,opt.Z,reshape(opt.utility,size(opt.X)), 'FaceColor','interp', 'EdgeColor','interp','Marker','.');
imagesc(flip(opt.mapWithObstacles))
axis tight
axis equal
% colorbar;
%plot regions to avoid
% plot(x1+r*sin(ang),y1+r*cos(ang),'w','LineWidth',2); % r is the radius
% plot(x2+r*sin(ang),y2+r*cos(ang),'w','LineWidth',2); % r is the radius
axis equal
axis([ opt.DomainBounds.xmin opt.DomainBounds.xmax opt.DomainBounds.ymin opt.DomainBounds.ymax])
view(0,90)
title('Candidate trajectories')
figure(1);
subplot(1,3,3);
set(gca,'FontSize',20);    hold on
% opt.htraj=color_line3(opt.xss(:,1), opt.xss(:,2), opt.xss(:,3),opt.traj_stat(:),'.');
opt.htraj = surface(opt.X,opt.Y,opt.Z,reshape(opt.traj_stat(:),size(opt.X)), 'FaceColor','interp', 'EdgeColor','interp','Marker','.');
% if (strcmp(opt.algorithm,'KL') && strcmp(opt.sensorMode,'largeFootPrint'))
    caxis([0 10])
% end

title('Time-Average Statistics')
axis equal
axis([ opt.DomainBounds.xmin opt.DomainBounds.xmax opt.DomainBounds.ymin opt.DomainBounds.ymax])
view(0,90)

%%  

for iagent = 1:opt.nagents
    opt.iagent = iagent;
    opt.figg = figure(1);
    subplot(1,3,2);
    hold on
    scatter3(opt.agent(iagent).xi(1),opt.agent(iagent).xi(2),0,30,'filled','MarkerFaceColor',opt.colors{iagent},'MarkerEdgeColor',[1,1,1])
    opt.agent(iagent).trajFigOPTIMAL =  draw_path(traj(0*opt.z, opt), opt.colors{iagent}, 2, 5);
    
    subplot(1,3,1);hold on

    opt.agent(iagent).trajFig =  draw_path(traj(0*opt.z, opt), opt.colors{iagent}, 2, 5);
    scatter3(opt.agent(iagent).xi(1),opt.agent(iagent).xi(2),0,30,'filled','MarkerFaceColor',opt.colors{iagent},'MarkerEdgeColor',[1,1,1])
end
opt.ceFig = draw_path(0*rand(3,3), 'b',3, 5);    %plot trajectories inside cem



end

