function [X,Y,G] = GenerateUtilityMap(opt,addnoise)
xdel=1;%resolution in x
ydel=1;%resolution in y
xRange=opt.DomainBounds.xmin:xdel:opt.DomainBounds.xmax-xdel;
yRange=opt.DomainBounds.ymin:ydel:opt.DomainBounds.ymax-ydel;

[X,Y] = meshgrid(xRange,yRange);

if (addnoise==1)
    n=0.05;
else
    n=0;
end

%% infotrmation map presnted in howie's office last time
m=[50 50];
s=25*eye(2);
m2=[100 90];
s2=100*eye(2);
m3=[60 100];
s3=60*eye(2);

%%new map
% m=[100 200];
% s=150*eye(2);
% m2=[220 200];
% s2=800*eye(2);
% m3=[120 120];
% s3=600*eye(2);
G1 = mvnpdf([X(:), Y(:)],m,s);
G2 = mvnpdf([X(:), Y(:)],m2,s2);
G3 = mvnpdf([X(:), Y(:)],m3,s3);
G=(G1+3*G2+G3);
% noise=n*rand(size(G));
% G=G+noise;
G=max(G,0); %crop below 0
G=G./max(G); %normalize
G = G./sum(sum(G));

end

