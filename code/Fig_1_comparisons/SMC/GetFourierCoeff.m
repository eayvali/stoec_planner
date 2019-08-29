function [muk, HK] = GetFourierCoeff(opt,X,Y)
%% returns the fourier coeffients of the prior information distribution
mu=opt.erg.mu;
Nkx=opt.erg.Nkx;
Nky=opt.erg.Nky;
Lx=opt.L(1);
Ly=opt.L(2);

% Initializing Fourier coefficents of prior
muk=zeros(Nkx,Nky);

HK = sqrt(Lx*Ly) .* [1 sqrt(1/2)*ones(1,Nky-1)]' * [1 sqrt(1/2)*ones(1,Nkx-1)];
mu = reshape(mu,size(X));

%% using matlab DCT
% muk = dct(mu);

%% Our implementation
for kx = 0:Nkx-1
    for ky = 0:Nky-1
        muk(kx+1, ky+1) = sum(sum( mu .* cos(kx * pi * X/Lx) .* cos(ky * pi *  Y/Ly) )) / HK(ky+1,kx+1);
    end
end


% opt.erg.muk = muk;
% for kx = 0:Nkx-1
%     for ky = 0:Nky-1
%               
%         for xRange=0:1:Lx-1
%             for yRange=0:1:Ly-1
%                 muk(kx+1, ky+1) = muk(kx+1, ky+1)+ mu(uint8(xRange*1+1),uint8(yRange*1+1)) *(1/HK(kx+1,ky+1))*cos(kx * pi * xRange/Lx) * cos(ky * pi * yRange/Ly);
%             end
%         end       
%         
%     end
% end


