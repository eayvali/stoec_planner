function [ traj_stat ] = timeAverageStatisticsDistribution( traj, opt )
%returns time average statistics distribution, distcrete pdf same size as
%information map(utility distribution)

% input:
      % -  traj: n x 2 matrix where is the number of data points on the
      % trajectoy and 2 are (x,y) of each data point

ind = knnsearch(opt.kdOBJ, traj(:,1:2));
traj_stat = accumarray(ind,1,size(opt.erg.mu(:)));
traj_stat=traj_stat./sum(sum(traj_stat));%normalizing

end

