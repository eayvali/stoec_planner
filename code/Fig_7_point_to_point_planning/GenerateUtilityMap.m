function [X,Y,G] = GenerateUtilityMap(opt,addnoise)
xdel=1;
ydel=1;
xRange=opt.DomainBounds.xmin:xdel:opt.DomainBounds.xmax-xdel;
yRange=opt.DomainBounds.ymin:ydel:opt.DomainBounds.ymax-ydel;

[X,Y] = meshgrid(xRange,yRange);
if (addnoise==1)
    n=0.05;
else
    n=0;
end

%% 
m=[40 40];
s=75*eye(2);
m2=[100 70];
s2=500*eye(2);
m3=[60 100];
s3=200*eye(2);

G1 = 0*mvnpdf([X(:), Y(:)],m,s);
G2 = mvnpdf([X(:), Y(:)],m2,s2);
G3 = 0*mvnpdf([X(:), Y(:)],m3,s3);
G=(G1+3*G2+G3);
% noise=n*rand(size(G));
% G=G+noise;
G=max(G,0); %crop below 0
G=G./max(G); %normalize
G = G./sum(sum(G));
%%
% % Uniform map
% G = zeros(size(X));
% G(21:130,21:130) = ones(110,110);
% G = G./sum(sum(G));

end
