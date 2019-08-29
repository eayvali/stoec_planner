function [ BhattDistance ] = evaluateBhattacharyyaDist( traj_stat, opt )

%https://en.wikipedia.org/wiki/Bhattacharyya_distance
BC = sum(sqrt(traj_stat.*opt.erg.mu(:)));
BhattDistance = -log(BC);

end

