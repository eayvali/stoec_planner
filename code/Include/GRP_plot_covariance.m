% clc;close all;clear;
load('exp_data_old.mat')
% load('exp_data_alternative.mat')
mesh = pointCloud2mesh(gt, [0 0 1],5);
trianglePoints = mesh.vertices;
triangleFaces = mesh.triangles;
numTriangles = length(triangleFaces(:,1));
figure
hPyramid = trisurf(triangleFaces, trianglePoints(:,1), trianglePoints(:,2), trianglePoints(:,3),'FaceColor','b','FaceAlpha',0.2);
axis equal

%% Interpolate over the mesh to obtain a densed point cloud
[zgrid,xgrid,ygrid] = gridfit(gt(:,1),gt(:,2),gt(:,3),min(gt(:,1)):1:max(gt(:,1)),min(gt(:,2)):1:max(gt(:,2)));
xsGT = meshgrid2vec(xgrid, ygrid, zgrid); %3D point cloud
xsGT=xsGT';
scatter3(xsGT(:,1), xsGT(:,2), xsGT(:,3),'k*');
axis equal
datafull=[xsGT,yGT];
figure
color_line3(xsGT(:,1), xsGT(:,2), xsGT(:,3),yGT,'.')
data=[gt,c];
datainit=datafull(idxinit,:);
x=datainit(:,1:3)
y=datainit(:,4);

%% Batch Random Data
%  Generate x,y,xstar for batch random data
close all
cnt=200;
ebar_coef=3;
pcolor= 'mo';
ys2save=[];
ymusave=[];
xssave=[];

covfunc = @covSEiso; likfunc = @likGauss;
sn = 0; ell =3; sf = sqrt(1);
hyp.lik = log(sn); hyp.cov = log([ell; sf]);
[ymu ys2 fmurnd fs2rnd]= gp(hyp, @infExact, [], covfunc, likfunc, x, y, xsGT);
ymu(ymu<0)=0;

set(gcf,'color','w');
axis([-40 60 -40 60  -80 -50])
% plot3d_errorbars3(xsGT(:,1),xsGT(:,2),xsGT(:,3),ymu,ys2)
tri=delaunay(xsGT(:,1),xsGT(:,2));
trisurf(tri,xsGT(:,1),xsGT(:,2),xsGT(:,3),ymu);
view(0,90)
lighting phong
shading interp
grid
axis equal
hold on 
scatter3(x(:,1),x(:,2),x(:,3),10,'filled',pcolor)
%%
x=xsGT(:,1);
y=xsGT(:,2);
z=xsGT(:,3);
e=4*ys2;
% now draw the vertical errorbar for each point
for i=1:length(x)
xV = [x(i); x(i)];
yV = [y(i); y(i)];
zMin = z(i) + e(i);
zMax = z(i) - e(i);

zV = [zMin, zMax];
% draw vertical error bar
h=plot3(xV, yV, zV, '-k');
set(h, 'LineWidth', 1);
end
