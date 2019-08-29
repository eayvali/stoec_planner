function [ pose,opt ] = initialization()
%Initialized the coverage problem. Domain, speed limits, number of agents...

opt=[];
%% Domain bounds
DomainBounds.xmin = 0.0;
DomainBounds.xmax = 150.0;
DomainBounds.ymin = 0.0;
DomainBounds.ymax = 150.0;
Lx = DomainBounds.xmax - DomainBounds.xmin;
Ly = DomainBounds.ymax - DomainBounds.ymin;

opt.DomainBounds = DomainBounds;
opt.L = [Lx;Ly];

%% Agents params: Velocity Bounds, number of agents... 

% number of agents
opt.nagents = 1;

% forward velocity lower bound
if ~isfield(opt, 'vlb')
    opt.vlb = .1*ones(opt.nagents,1);
end

% forward velocity upper bound
if ~isfield(opt, 'vub')
    opt.vub = 5*ones(opt.nagents,1);
end


% angular velocity lower bound
if ~isfield(opt, 'wlb')
    opt.wlb = -0.2*ones(opt.nagents,1);
end


% angular velocity upper bound
if ~isfield(opt, 'wub')
    opt.wub = 0.2*ones(opt.nagents,1);
end


%% Initializing agent locations
pose.x = [140,100,25];
pose.y = [20,40,125];
pose.theta = 120*pi/180*ones(opt.nagents,1);
%% ergodicity params

opt.erg.s=1.5;%parameter of soblev norm

opt.erg.Nkx = 50;
opt.erg.Nky = 50;
%%%used in calculation CK and Bj in SMC update
opt.erg.KX = (0:opt.erg.Nkx-1)' * ones(1,opt.erg.Nky);
opt.erg.KY = ones(opt.erg.Nkx,1) * (0:opt.erg.Nky-1);
opt.erg.LK = 1.0 ./ ((1.0 + opt.erg.KX.^2 + opt.erg.KY.^2).^opt.erg.s);
%%%
opt.erg.HK=[];% normalizer of fourier basis functions, will ber assigned in GetFourierCoeff and used in multiple places
opt.erg.muk=[];

%% simulation params
opt.sim.Nsteps = 10000;
opt.sim.dt = 0.1;

end

