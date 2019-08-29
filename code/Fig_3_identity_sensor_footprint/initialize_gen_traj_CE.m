function [opt] = initialize_gen_traj_CE(opt)
%% domain bounds 
DomainBounds.xmin = 0.0;
DomainBounds.xmax = 150.0;
DomainBounds.ymin = 0.0;
DomainBounds.ymax = 150.0;
Lx = DomainBounds.xmax - DomainBounds.xmin;
Ly = DomainBounds.ymax - DomainBounds.ymin;

opt.DomainBounds = DomainBounds;

%dimension of the search space
if ~isfield(opt, 'dim')
    opt.dim = 3;
end

% workspace lower bound
if ~isfield(opt, 'xlb')
    opt.xlb = [DomainBounds.xmin;DomainBounds.ymin];
end

% workspace upper bound
if ~isfield(opt, 'xub')
    opt.xub = [DomainBounds.xmax;DomainBounds.ymax];
end

% grid cells along each dimension
if ~isfield(opt, 'ng')
    opt.ng = [Lx;Ly];
end

% trajectory parameters:how many primitives
if ~isfield(opt, 'sn')
    opt.sn = 5;
end

% time-step
if ~isfield(opt, 'dt')
    opt.dt = 0.1;
end

% time horizon (seconds)
if ~isfield(opt, 'tf')
    opt.tf = 40;
end

% forward velocity lower bound
if ~isfield(opt, 'vlb')
    opt.vlb = .1;
end

% forward velocity upper bound
if ~isfield(opt, 'vub')
    opt.vub = 5;
end


% angular velocity lower bound
if ~isfield(opt, 'wlb')
    opt.wlb = -0.2;
end


% angular velocity upper bound
if ~isfield(opt, 'wub')
    opt.wub = 0.2;
end

% Cross-entropy parameters

% outer CE iterations (only useful if we want to display/save each CE iteration)
if ~isfield(opt, 'iters')  
    opt.iters = 1;
end

if ~isfield(opt, 'ce')
    opt.ce = [];
end

% #of samples
if ~isfield(opt.ce, 'N')
    opt.ce.N = 40;
end

% smoothing parameter
if ~isfield(opt.ce, 'v')
    opt.ce.v = .8;
end

% CE iterations per optimization
if ~isfield(opt.ce, 'iter')
    opt.ce.iter = 6;
end

% use the sigma-point CE
if ~isfield(opt.ce, 'sigma')
    opt.ce.sigma = 0;
end

% initial covariance
if ~isfield(opt.ce, 'C0')
    opt.ce.C0 = diag(repmat([1;1],opt.sn,1));
end

% initial mean (straight line)
if ~isfield(opt.ce, 'z0')
    opt.ce.z0 = repmat([1; 0], opt.sn,1);
end

% how many receding horizon
if ~isfield(opt, 'stages')
    opt.stages = 10;
end

%CE weight:elite bias?
if ~isfield(opt, 'devBias')
    opt.devBias = .001;
end

% obstacle map
if isfield(opt, 'mapFile') 
    opt.map = double(imread(opt.mapFile))/255;
else
    opt.map = [];
end

%plot candidate trajectories inside cem
if ~isfield(opt, 'ceFlag')
    opt.ceFlag = 0;
end

opt.z = opt.ce.z0;
zlb = repmat([opt.vlb; opt.wlb], opt.sn,1);
zub = repmat([opt.vub; opt.wub], opt.sn,1);
opt.ce.z = opt.ce.z0;
opt.ce.C = opt.ce.C0;
opt.ce.lb = zlb;
opt.ce.ub = zub;
end