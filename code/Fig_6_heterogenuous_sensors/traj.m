function xs = traj(z, opt)
tl = opt.tf/opt.sn;
xs = opt.agent(opt.iagent).xi;

for i=1:opt.sn,
    v = z(2*(i-1) + 1);
    w = z(2*(i-1) + 2);
    th = xs(opt.dim+1,end);
    
    t = opt.dt:opt.dt:tl;
    if(opt.dim == 3)
        if (abs(w) < 1e-10)
            xs = [xs(1,:), xs(1,end) + t*v*cos(th);
                xs(2,:), xs(2,end) + t*v*sin(th);
                xs(3,:), xs(3,end) + t*0;
                xs(4,:), xs(opt.dim+1,end) + t*0];
            
        else
            xs = [xs(1,:), xs(1,end) + v/w*(sin(th + t*w) - sin(th));
                xs(2,:), xs(2,end) + v/w*(-cos(th + t*w) + cos(th));
                xs(3,:), xs(3,end) + t*0;
                xs(4,:), xs(opt.dim+1,end) + t*w];
        end
    else
        if (abs(w) < 1e-10)
            xs = [xs(1,:), xs(1,end) + t*v*cos(th);
                xs(2,:), xs(2,end) + t*v*sin(th);
                xs(3,:), xs(opt.dim+1,end) + t*0];
            
        else
            xs = [xs(1,:), xs(1,end) + v/w*(sin(th + t*w) - sin(th));
                xs(2,:), xs(2,end) + v/w*(-cos(th + t*w) + cos(th));
                xs(3,:), xs(opt.dim+1,end) + t*w];
        end
    end
end
xs(4,:) = wrapTo2Pi(xs(4,:)); 
%plot traj in cem
if opt.ceFlag==1;
    set(opt.ceFig,'XData', xs(1,:));
    set(opt.ceFig,'YData', xs(2,:));
    set(opt.ceFig,'ZData', xs(3,:));
    drawnow
end
end