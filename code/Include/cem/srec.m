function f = srec
%Demonstration of Receding Horizon Adaptive Sampling for
%discovering peak concentration in a 2d scalar field

clear
N0 = 25;

if 0
opt.figs(1) = figure;
opt.figs(2) = figure;
opt.figs(3) = figure; 
opt.figs(4) = figure;
else
  opt.figs = [];  
end

opt.dr = 5;

%opt.xi = [-45; -45; pi/4];
opt.xi = [25; 25; 1.1*pi];

opt.xr = -50:opt.dr:50;
opt.yr = -50:opt.dr:50;
opt.xlb = min(opt.xr);
opt.xub = max(opt.xr);
opt.ylb = min(opt.yr);
opt.yub = max(opt.yr);

opt.xmin = [];
opt.fmin = [];
opt.J = [];
opt.cP = [];
opt.fP = [];
opt.sel = 'pi';

opt.sn = 5; 
opt.dt = 3;
opt.tf = 30;

opt.I=[];
%opt.I = double(imread('conc.ppm'))/255;
%opt.I = double(imread('do1.ppm'))/255;
opt.I = 1.5*double(imread('do1.ppm'))/255;

%opt.I =;
%size(I)
%I(250,250,:)

%return

[X,Y] = meshgrid(opt.xr, opt.yr);
% list of grid points
xss = [reshape(X, 1, size(X,1)*size(X,2));
       reshape(Y, 1, size(Y,1)*size(Y,2))];

opt.fs = fun(xss,opt);

p = randperm(size(xss,2));

opt.pss = prior(xss, opt);

% initial data
%xs = xss(:,p(1:N0));
%xs = [.2, .1, .2, .1;
%      .1, -.3, .3, -.3];


%xs = [.2, .5, .6, .9;
%      .3, -.4, .3, -.3];

xs=opt.xi(1:2);


%xs = [1, 1, -1, -1;
%      1, -1, 1, -1];

%xs = repmat(opt.xi(1:2), 1, 5) + .1*randn(2,5);


%xss = xss(sort(p(N0+1:end)));
  
fs = zeros(1,size(xs,2));
for i=1:size(xs,2),
  fs(i) = fun(xs(:,i),opt)
end


% cost function gp
fopts.l = 5;

if ~isempty(opt.I)
  fopts.s = .4;
  fopts.sigma = .001;
else
  fopts.s = 20;
  fopts.sigma = .01;
end

fgp = gp_init(xs, fs, fopts)
fgp.fun = @fun;

% test points
%Nmax = 500;

% init opt
[y,ind] = min(fs);
opt.xmin = fgp.xs(:,ind);
opt.fmin = y;
opt.xss = xss;
opt.fgp = fgp;

N = 50;

%plot(S.xss, S.fss, '.b')
%ofig = figure

gp_plot(opt)

%xfig = figure
%plot_env(opt)

% forward velocity lower bound
vlb = .1;
% forward velocity upper bound
vub = 5;
% angular velocity lower bound
wlb = -.5;
% angular velocity upper bound
wub = .5;

%z = zeros(2*opt.sn, 1);
z0 = repmat([3; 0], opt.sn,1);
z=z0;

zlb = repmat([vlb;wlb], opt.sn,1);
zub = repmat([vub;wub], opt.sn,1);

xs = traj(z0,opt);

f = traj_cost(z0,opt)

%[z,fval,exitflag,output] = fmincon(@(z)traj_cost(z,opt), z, [], [],[],[],zlb,zub);

xs = traj(z,opt)

opts.v = .8;
opts.iter = 2;
opts.N = 100;
opts.sigma = 0;

iters = 10;
mu = z;
if ~isempty(opt.I)
  C0 = diag(repmat([1;1],opt.sn,1));
else
  C0 = diag(repmat([1;.4],opt.sn,1));
end
cs = zeros(iters,1);
opts.C = C0;

opts.lb = zlb;
opts.ub = zub;

% traveled path
xps=opt.xi;

video =0;
mov = 0;
if video,
  mov = VideoWriter('as.avi');
  open(mov);
  
%  mov = avifile('as.avi');
%  mov.Quality = 100;
%  mov.fps = 2*iters;
end

j=1;

for k=1:500
  
  %% PLANNING
  opts.C = .5*opts.C + .5*C0;
  subplot(2,2,1);
  Gp=  draw_path(traj(z,opt), 0, 'r',2,5);
  z = z0;

  for i=1:iters
    [z, c, mu, C] = cem(@traj_cost, z, opts, opt);
    x = mu;
    cs(i) = c;
    opts.C = C;
    xs = traj(z,opt);
    
    set(Gp,'XData', xs(1,:));
    set(Gp,'YData', xs(2,:));
    drawnow
    if video,
      saveas(gcf,['as/as' num2str(j,'%03d') '.jpg'])
      j=j+1;
      %    saveas(gca,['se2opt/v' num2str(c) '.eps'],'psc2');
      %    mov = addframe(mov,getframe(gca));
      %    writeVideo(mov,getframe);
      %    print(gcf,'-dpng', 'figures/env.png')
    end        
  end
  
  % "EXECUTE" START OF PATH
  x = xs(:,2);
  opt.xi = x;
  
  % GET MEASUREMENT AND ADD TO GP
  xps = [xps, x];
  
  opt.fgp = gp_add(opt.fgp, x(1:2), fun(x(1:2), opt));
  
%  fun(x(1:2),opt);
  
  hold off
  gp_plot(opt)
  hold on
  
  subplot(2,2,1)
  draw_path(xs(1:2,2:end), 0, 'b',3, 5);
  hold on
  draw_path(xps(1:2,:), 0, 'g',3, 5);
  xlabel('m')
  ylabel('m')
  drawnow
  
  subplot(2,2,3)
  hold off
  contour(opt.xr, opt.yr, reshape(opt.pss, length(opt.xr), ...
                                  length(opt.yr)));
  
  %h = contour(opt.xr, opt.yr, reshape(ms, length(opt.xr), ...
  % length(opt.yr)));
  
  hold on
  draw_path(xs(1:2,2:end), 0, 'b',2, 5);
  draw_path(xps(1:2,:), 0, 'g',3, 5);
  axis square
  axis equal
  axis([min(opt.xr),max(opt.xr),min(opt.yr),max(opt.yr)]  )
  title('Executed (green) and Planned (blue) paths; with prior contours')
  xlabel('m')
  ylabel('m')
  
end

if video,
  close(mov);
end


return


for i=1:60,
  
  [opt, x, y] = gp_select(opt)
  
%  if (abs(x-cgp.xs)<.005)
%    continue;
%  end
    
  f = fun(x, opt);
  
  if f < opt.fmin
    opt.fmin = f;
    opt.xmin = x;
  end
  
  opt.fgp = gp_add(opt.fgp, x, f);
%  cgp = cgp_add(cgp, x, c);
  
  display('opt:') 
  opt.xmin

%  figure(ofig)
  gp_plot2(opt)
  
  figure(xfig)
  plot_env(opt)
  print(gcf,'-dpng', 'figures/env.png')
  
%  plot(x,f,'ob')

%  pause(3)
%  S.xmin
 % S.fmin
end

%plot(S.xss, S.ms, '.r')

function G = draw_path(xs, z, c, lw, ms)

G=plot3(xs(1,:), xs(2,:), z*ones(size(xs,2),1), [c '-'], ...
        'LineWidth', lw, 'MarkerSize', ms);

function f = fun(xs, opt)

if ~isempty(opt.I)
  xr = opt.xub-opt.xlb;
  yr = opt.yub-opt.ylb;
  is = floor((opt.yub - xs(2,:))/yr*size(opt.I,2)) + 1;
  js = floor((xs(1,:)-opt.xlb)/xr*size(opt.I,1)) + 1;
  
  is(find(is>size(opt.I,1)))=size(opt.I,2);
  js(find(js>size(opt.I,2)))=size(opt.I,2);
  is(find(is<1))=1;
  js(find(js<1))=1;
  for i=1:size(xs,2)
    f(i)= opt.I(is(i),js(i),1);
  end
  return
end

%mvnpdf([0, 0], [0, 0], diag([.05, .1])) 
f = 100000*mvnpdf(xs', [0, 0], diag([400, 400])); 
f = f + 20000*mvnpdf(xs', [20, 20], diag([50, 100])); 
f = f + randn(size(f))*.04;


function f = prior(xs, opt)

%mvnpdf([0, 0], [0, 0], diag([.05, .1])) 

if ~isempty(opt.I)
  f = mvnpdf(xs', [0, 0], diag([1000, 1000]));
else
  f = mvnpdf(xs', [0, 0], diag([1000, 1000]));
end

f = f/max(f);

function xs = sample(N, opt)
xs = 2.5 - 5*rand([2, N]);


function xs = traj(z, opt)

tl = opt.tf/opt.sn;

xs = zeros(3, 1);
xs = opt.xi;

for i=1:opt.sn,
  v = z(2*(i-1) + 1);
  w = z(2*(i-1) + 2);
  th = xs(3,end);

  t = opt.dt:opt.dt:tl;
  
  if (abs(w) < 1e-10)     
    xs = [xs(1,:), xs(1,end) + t*v*cos(th);
          xs(2,:), xs(2,end) + t*v*sin(th);
          xs(3,:), xs(3,end) + t*0];

  else
    xs = [xs(1,:), xs(1,end) + v/w*(sin(th + t*w) - sin(th));
          xs(2,:), xs(2,end) + v/w*(-cos(th + t*w) + cos(th));
          xs(3,:), xs(3,end) + t*w];
  end  
end

function xs = traj2(z, opt)

dt = opt.tf/opt.sn;
xs = zeros(3, opt.sn+1);
xs(:,1) = opt.xi;

for i=1:opt.sn,
  v = z(2*(i-1) + 1);
  w = z(2*(i-1) + 2);
  
  th = xs(3,i);
  if (abs(w) < 1e-10)     
    xs(:,i+1) = xs(:,i) + dt*[v*cos(th);
                    v*sin(th);
                    0];

  else
    xs(:,i+1) = xs(:,i) + [v/w*(sin(th + dt*w) - sin(th));
                    v/w*(-cos(th + dt*w) + cos(th));
                    dt*w];    
  end  
end



function f = traj_cost(z, opt)

%gp = opt.fgp;

xs = traj(z, opt);

[ms, ss] = gp_predict(opt.fgp, xs(1:2,2:end));
vs = sqrt(diag(ss));

ps = prior(xs(1:2,2:end), opt);
%ps = ones(size(ps));

if ~isempty(opt.I)
  f = -sum(ps.*(ms + 1.96*vs));

  f = -sum(ms + 1.96*vs);
%  f = -sum(ms);
else
%  f = -sum(ps.*(ms + 1.96*vs));
%f = sum(ps.*(1.96*vs)) - sum(ps);
f = -sum(vs) - sum(ps);
end

%f = sum(ps.*(ms));
%f = -sum(ps.*vs);

vs = z(1:2:end-1);
ws = z(2:2:end);
dws = ws(2:end)-ws(1:end-1);

%f = f*(1 + .01*ws'*ws);% + .5*dws'*dws);

%f = f + .001*vs'*vs;

%f = mean(ms);

%fs = fun(xs, opt);

%for i=1:size(xs,2)%
%  gp = gp_add(gp, xs(:,i), 
%end


function f = plot_env(opt)

return

if ~isempty(opt.xmin)
  xs = [opt.xi, opt.xmin];
  xs = interp1(linspace(0, 1, size(xs,2)), xs', ...
               linspace(0, 1, opt.snf), 'cubic')';
  
  plot3(opt.xmin(1), opt.xmin(2), 0, '*','MarkerSize', 2, 'LineWidth',5)
  plot3(xs(1,:), xs(2,:), zeros(size(xs,2),1), '-g', 'LineWidth',5);
end

for i=1:size(opt.fgp.xs,2)
%  xs = [opt.xi, opt.fgp.xs(:,i), opt.xf];
%  xs = interp1(linspace(0, 1, size(xs,2)), xs', ...
%               linspace(0, 1, opt.snf), 'cubic')';
  
  plot3(opt.fgp.xs(1,i), opt.fgp.xs(2,i), zeros(size(opt.fgp.xs,2),1), ...
        'o','MarkerSize',5,  'LineWidth',2);
  %  plot3(xs(1,:), xs(2,:), zeros(size(xs,2),1), '--', 'LineWidth',2); 
end

axis square
%axis equal
view(-20,48)
%view(3)



function f = gp_plot(opt)

set(0,'DefaultAxesFontName', 'Times New Roman')
set(0,'DefaultAxesFontSize', 15)

set(0,'DefaultTextFontname', 'Times New Roman')
set(0,'DefaultTextFontSize', 15)

if (~isempty(opt.figs))
  figure(opt.figs(1));
  set(opt.figs(1), 'Position', [100, 100, 800, 600]);
else
  sp = subplot(2,2,1);
  set(gcf, 'Position', [100, 100, 1400, 800]);
end

[ms, ss] = gp_predict(opt.fgp, opt.xss);
vs = sqrt(diag(ss));

h = surf(opt.xr, opt.yr, reshape(ms, length(opt.xr), length(opt.yr)), ...
          'FaceColor','interp','FaceLighting','phong');
hold on
h1 = surf(opt.xr, opt.yr, reshape(ms + 1.96*vs, length(opt.xr), length(opt.yr)), ...
          'FaceColor','interp','FaceLighting','phong','EdgeAlpha',.3);
%set(h,'FaceAlpha',0);
h2 = surf(opt.xr, opt.yr, reshape(ms - 1.96*vs, length(opt.xr), length(opt.yr)), ...
          'FaceColor','interp','FaceLighting','phong','EdgeAlpha',.3);
%set(h,'FaceAlpha',0);

xlabel('m')
ylabel('m')

alpha(h,.8)
alpha(h1,.3)
alpha(h2,.3)

%xlabel('$x_1$','Interpreter','latex')
%%ylabel('$x_2$','Interpreter','latex')
%zlabel('$\mathbb{E}[J(x)] \pm \beta Var[J(x)]$', 'Interpreter', 'latex')

view(-20,48)
axis tight
%axis equal
title('Guassin Process Model (+-95% conf.)')
drawnow

%print(gcf,'-dpng', 'figures/Jx.png')

%savesp(sp, 'figures/Jx');

%plot(xts, ms + 1.96*vs, '--', xts, ms - 1.96*vs, '--', xts, ms, '-');
%hold on
%plot3(opt.fgp.xs(1,:), opt.fgp.xs(2,:), opt.fgp.fs, 'or');

%title('Probabilistic Model of Trajectory Cost $J(x)$')


if (~isempty(opt.figs))
  figure(opt.figs(2));
  set(opt.figs(2), 'Position', [100, 100, 800, 600]);
else
  sp = subplot(2,2,2);
end

cns = 1;

hold off
if ~isempty(opt.fs)
  if cns
    h = contour(opt.xr, opt.yr, reshape(opt.fs, length(opt.xr), ...
                                        length(opt.yr)));
    hold on
    plot(opt.fgp.xs(1,:), opt.fgp.xs(2,:), '-g','LineWidth',2); 
  else
    h = surfc(opt.xr, opt.yr, reshape(opt.fs, length(opt.xr), length(opt.yr)),...
              'FaceColor','interp','FaceLighting','phong','EdgeAlpha',.3);
    
    set(h,'FaceAlpha',0.7);
    view(-20,48)
  end

  axis tight
%  axis equal
  %  plot(gp.xss, gp.J, 'b')
  hold on
end
title('True Scalar Field')
xlabel('m')
ylabel('m')

if (~isempty(opt.figs))
  figure(opt.figs(3));
  set(opt.figs(3), 'Position', [100, 100, 800, 600]);
else
  sp = subplot(2,2,4);
end

hold off

if cns
  h = contour(opt.xr, opt.yr, reshape(ms, length(opt.xr), ...
                                      length(opt.yr)));
  hold on
  plot(opt.fgp.xs(1,:), opt.fgp.xs(2,:), 'ok-'); 
else
  h = surf(opt.xr, opt.yr, reshape(ms, length(opt.xr), length(opt.yr)), ...
           'FaceColor','interp','FaceLighting','phong');
  %alpha(h,.5)
  set(h,'FaceAlpha',0.5);
  %  plot(gp.xss, gp.J, 'b')
  hold on
  plot3(opt.fgp.xs(1,:), opt.fgp.xs(2,:), opt.fgp.fs(:),'*'); 
  view(-20,48)

end

axis tight
%axis equal
xlabel('m')
ylabel('m')
title('Estimated scalar Field (with marked samples)')

%title('States Sampling Distribution')

%xlabel('$x_1$','Interpreter','latex')
%ylabel('$x_2$','Interpreter','latex')
%zlabel('$P(x)$', 'Interpreter', 'latex')


%print(gcf,'-dpng', 'figures/Px.png')
%savesp(sp, 'figures/Px');


if (~isempty(opt.figs))
  figure(opt.figs(4));
  set(opt.figs(4), 'Position', [100, 100, 800, 600]);
else
%  sp = subplot(2,2,3);
end



hold off
if ~isempty(opt.cP)
  
  h = surfc(opt.xr, opt.yr, reshape(opt.cP, length(opt.xr), length(opt.yr)),...
  'FaceColor','interp','FaceLighting','phong');
  
  %  set(h,'FaceAlpha',0);
  view(-20,48)
axis tight
%axis equal
  
  hold on
end
%title('Constraint Satisfaction Probability')

%zlabel('$P(g(x)\geq 0)$', 'Interpreter', 'latex')
%xlabel('$x_1$', 'Interpreter', 'latex')
%ylabel('$x_2$', 'Interpreter', 'latex')

%print(gcf,'-dpng', 'figures/Pg.png')
%savesp(sp, 'figures/Pf');