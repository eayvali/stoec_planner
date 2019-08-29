function [Ergodicity_Metric]= EvaluateErgodicity(xs,opt)

DomainBounds.xmin = opt.xlb(1);
DomainBounds.xmax = opt.xub(1);
DomainBounds.ymin = opt.xlb(2);
DomainBounds.ymax = opt.xub(2);

Lx = DomainBounds.xmax - DomainBounds.xmin;
Ly = DomainBounds.ymax - DomainBounds.ymin;

Nkx = size(opt.erg.muk, 1);
Nky = size(opt.erg.muk, 2);

%------------Calculate Ck------------%
Ck = zeros(opt.erg.Nkx, opt.erg.Nkx);
% Nsteps=opt.tf/opt.dt;%%BUG
Nsteps=size(xs,2);
HK = sqrt(Lx*Ly) .* [1 sqrt(1/2)*ones(1,Nkx-1)]' * [1 sqrt(1/2)*ones(1,Nky-1)];
KX = (0:Nkx-1)' * ones(1,Nky);
KY = ones(Nkx,1) * (0:Nky-1);

for it = 1:Nsteps %compatible with the length of xs: Nsteps+1
    % Updating Fourier Coefficients of Coverage Distribution
    xrel = xs(1,it) -  DomainBounds.xmin;
    yrel = xs(2,it) -  DomainBounds.ymin;

    Ck = Ck + cos(KX * pi * xrel/Lx)./HK .* cos(KY * pi * yrel/Ly)*opt.dt;
end
    Ck = Ck + opt.erg.Ck;
[Ergodicity_Metric] = Calculate_Ergodicity(Ck/(Nsteps*opt.currentStage*opt.dt*opt.nagents), opt.erg.muk, DomainBounds,opt.nagents);
