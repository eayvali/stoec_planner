function G = draw_path(xs, c, lw, ms)
if(size(xs,1) == 2)
    G=plot3(xs(1,:), xs(2,:), xs(3,:), [c '-'], ...
        'LineWidth', lw, 'MarkerSize', ms);
else
    G=plot3(xs(1,:), xs(2,:), xs(3,:), [c '-'], ...
        'LineWidth', lw, 'MarkerSize', ms);
end
end