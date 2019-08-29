function [ opt ] = accumulate_CK( opt, xs )

DomainBounds.xmin = opt.xlb(1);
DomainBounds.xmax = opt.xub(1);
DomainBounds.ymin = opt.xlb(2);
DomainBounds.ymax = opt.xub(2);

Lx = DomainBounds.xmax - DomainBounds.xmin;
Ly = DomainBounds.ymax - DomainBounds.ymin;

Nkx = size(opt.erg.muk, 1);
Nky = size(opt.erg.muk, 2);

Nsteps=size(xs,2);
HK = sqrt(Lx*Ly) .* [1 sqrt(1/2)*ones(1,Nkx-1)]' * [1 sqrt(1/2)*ones(1,Nky-1)];
KX = (0:Nkx-1)' * ones(1,Nky);
KY = ones(Nkx,1) * (0:Nky-1);
Ck = zeros(opt.erg.Nkx, opt.erg.Nkx);
for it = 1:Nsteps %compatible with the length of xs: Nsteps+1
% Updating Fourier Coefficients of Coverage Distribution
xrel = xs(1,it) -  DomainBounds.xmin;
yrel = xs(2,it) -  DomainBounds.ymin;

Ck = Ck + cos(KX * pi * xrel/Lx)./HK .* cos(KY * pi * yrel/Ly)*opt.dt;
end
opt.erg.Ck = opt.erg.Ck + Ck;
    


end

