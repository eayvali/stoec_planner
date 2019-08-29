function [ opt  ] = plot_initial( opt )

%Plot utility and trajectories
figure(1); hold on
subplot(1,3,2)
opt.h_utility = color_line3(opt.xss(:,1), opt.xss(:,2), opt.xss(:,3),opt.utility,'.');
hold on
scatter(opt.agent(1).xi(1),opt.agent(1).xi(2),'r','fill');%plot posistion of agents initially
text(opt.agent(1).xi(1),opt.agent(1).xi(2),'\leftarrow Start','Color','r')
hold on
scatter(opt.agent(1).xi(1),140,'r','fill');%plot posistion of agents initially
text(opt.agent(1).xi(1), 140,'\leftarrow Destination','Color','r')

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
color_line3(opt.xss(:,1), opt.xss(:,2), opt.xss(:,3),opt.utility,'.');
%plot regions to avoid
% plot(x1+r*sin(ang),y1+r*cos(ang),'w','LineWidth',2); % r is the radius
% plot(x2+r*sin(ang),y2+r*cos(ang),'w','LineWidth',2); % r is the radius
axis equal
axis([ opt.DomainBounds.xmin opt.DomainBounds.xmax opt.DomainBounds.ymin opt.DomainBounds.ymax])
view(0,90)
title('Candidate trajectories')
figure(1);
subplot(1,3,3);
opt.htraj=color_line3(opt.xss(:,1), opt.xss(:,2), opt.xss(:,3),opt.traj_stat(:),'.');

if (strcmp(opt.algorithm,'KL') && strcmp(opt.sensorMode,'largeFootPrint'))
    caxis([0 10])
end

title('Spatial Statistics')
axis equal
axis([ opt.DomainBounds.xmin opt.DomainBounds.xmax opt.DomainBounds.ymin opt.DomainBounds.ymax])
view(0,90)

%%  

for iagent = 1:opt.nagents
    opt.iagent = iagent;
    figure(1);
    subplot(1,3,2);
    % h1=  draw_path(traj(z, opt), 'b', 2, 5);
    hold on
    opt.agent(iagent).trajFigOPTIMAL =  draw_path(traj(0*opt.z, opt), opt.colors(iagent), 2, 5);
    
    subplot(1,3,1);
    opt.agent(iagent).trajFig =  draw_path(traj(0*opt.z, opt), opt.colors(iagent), 2, 5);
end
opt.ceFig = draw_path(0*rand(3,3), 'b',3, 5);    %plot trajectories inside cem


end

