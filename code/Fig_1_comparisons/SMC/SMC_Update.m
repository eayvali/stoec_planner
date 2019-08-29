function [pose, Ck] = SMC_Update(pose, Ck, time, opt)
%% parameters needed from options(opt)
Lx = opt.L(1);
Ly = opt.L(2);
xmin = opt.DomainBounds.xmin;
ymin = opt.DomainBounds.ymin;
dt = opt.sim.dt;
Nagents = opt.nagents;

KX = opt.erg.KX;
KY= opt.erg.KY;
LK = opt.erg.LK;
HK= opt.erg.HK;
muk = opt.erg.muk;

%% 
%Calculating the fourier coefficients of time average statistics distribution 
for iagent = 1:Nagents 
    xrel = pose.x(iagent) - xmin;
    yrel = pose.y(iagent) - ymin;
    Ck = Ck + cos(KX * pi * xrel/Lx) .* cos(KY * pi * yrel/Ly) * opt.sim.dt ./ HK';
end

for iagent = 1:Nagents
    xrel = pose.x(iagent) - xmin;
    yrel = pose.y(iagent) - ymin;
    
    Bjx = sum(sum(LK./ HK' .* (Ck - Nagents*time*muk) .* (-KX *pi/Lx .*sin(KX * pi * xrel/Lx) .* cos(KY *pi * yrel/Ly))));
    Bjy = sum(sum(LK./ HK' .* (Ck - Nagents*time*muk) .* (-KY *pi/Ly .*cos(KX * pi * xrel/Lx) .* sin(KY *pi * yrel/Ly))));

%     Bjnorm = sqrt(Bjx*Bjx + Bjy*Bjy);
    GammaV = Bjx*cos(pose.theta(iagent)) + Bjy*sin(pose.theta(iagent));
    GammaW = -Bjx*sin(pose.theta(iagent)) + Bjy*cos(pose.theta(iagent));
    
    % Updating agent location based on SMC feedback control law
    if GammaV >= 0
        v = opt.vlb(iagent);
    else
        v = opt.vub(iagent);
    end
    
    if GammaW >= 0
        w = opt.wlb(iagent);
    else
        w = opt.wub(iagent);
    end
    
    %velocity motion model
    if(abs(w) < 1e-10 )
        pose.x(iagent) = pose.x(iagent) + v*dt*cos(pose.theta(iagent));   
        pose.y(iagent) = pose.y(iagent) + v*dt*sin(pose.theta(iagent));
    else
        pose.x(iagent) = pose.x(iagent) + v/w*(sin(pose.theta(iagent) + w*dt) - sin(pose.theta(iagent)));   
        pose.y(iagent) = pose.y(iagent) + v/w*(cos(pose.theta(iagent)) - cos(pose.theta(iagent)+ w*dt));    
    end
        pose.theta(iagent) = pose.theta(iagent)+ w*dt;
    
    %No Need for this! % reflecting agent in case it goes out of domain bounds
%      [pose.x(iagent),pose.y(iagent)] = reflect_agent(pose.x(iagent),pose.y(iagent), opt.DomainBounds);
end

end

function [agentx, agenty] = reflect_agent(agentx,agenty, DomainBounds)

    xmin = DomainBounds.xmin;
    xmax = DomainBounds.xmax;
    ymin = DomainBounds.ymin;
    ymax = DomainBounds.ymax;

    if agentx < xmin
        agentx = xmin + (xmin - agentx);
    end
    if agentx > xmax
        agentx = xmax - (agentx - xmax);
    end
    if agenty < ymin
        agenty = ymin + (ymin - agenty);
    end
    if agenty > ymax
        agenty = ymax - (agenty - ymax);
    end

end
