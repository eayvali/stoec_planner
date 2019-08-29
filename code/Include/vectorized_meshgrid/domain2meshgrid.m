function [X, Y, Z] = domain2meshgrid(domain, resolution)
%DOMAIN2MESHGRID(domain, resolution)   generate meshgrid on parallelepiped
%   [X, Y] = DOMAIN2MESHGRID(domain, resolution) creates the matrices
%   X, Y definining a meshgrid covering the 2D rectangular domain
%   domain = [xmin, xmax, ymin, ymax] with resolution = [nx, ny] points
%   per each coordinate dimension.
%
%   [X, Y, Z] = DOMAIN2MESHGRID(domain, resolution) results into a
%   meshgrid over a 3D parallelepiped domain.
%
% input (2D Case)
%   domain = extremal values of parallelepiped
%          = [xmin, xmax, ymin, ymax]
%   resolution = # points /dimension
%              = [nx, ny]
%
% output (2D case)
%   X = [ny x nx] matrix of grid point abscissas
%   Y = [ny x nx] matrix of grid point ordinates
%
% input (3D Case)
%   domain = [xmin, xmax, ymin, ymax, zmin, zmax]
%   resolution = [nx, ny, nz]
%
% output (3D case)
%   X = [ny x nx x nz] matrix of grid point abscissas
%   Y = [ny x nx x nz] matrix of grid point ordinates
%   Z = [ny x nz x nz] matrix of grid point coordinates
%
% See also DOMAIN2VEC, VEC2MESHGRID, MESHGRID2VEC, MESHGRID.
%
% File:      domain2meshgrid.m
% Author:    Ioannis Filippidis, jfilippidis@gmail.com
% Date:      2012.01.14 - 
% Language:  MATLAB R2011b
% Purpose:   generate meshgrid on domain given a resolution
% Copyright: Ioannis Filippidis, 2012-

%% check input
if size(domain, 1) ~= 1
    error('size(domain, 1) ~= 1')
end

ndim = size(domain, 2) /2;

if ~isint(ndim)
    error('Non-integer domain dimension.')
end

if ~isequal(size(resolution), [1, ndim] )
    error('size(resolution) ~= [1, ndim]')
end

if ndim == 2
    [X, Y] = linmeshgrid2d(domain, resolution);
elseif ndim == 3
    [X, Y, Z] = linmeshgrid3d(domain, resolution);
end

function [X, Y] = linmeshgrid2d(domain, resolution)
xmin = domain(1, 1);
xmax = domain(1, 2);

ymin = domain(1, 3);
ymax = domain(1, 4);

nx = resolution(1, 1);
ny = resolution(1, 2);

x = linspace(xmin, xmax, nx);
y = linspace(ymin, ymax, ny);

[X, Y] = meshgrid(x, y);

function [X, Y, Z] = linmeshgrid3d(domain, resolution)
xmin = domain(1, 1);
xmax = domain(1, 2);

ymin = domain(1, 3);
ymax = domain(1, 4);

zmin = domain(1, 5);
zmax = domain(1, 6);

nx = resolution(1, 1);
ny = resolution(1, 2);
nz = resolution(1, 3);

x = linspace(xmin, xmax, nx);
y = linspace(ymin, ymax, ny);
z = linspace(zmin, zmax, nz);

[X, Y, Z] = meshgrid(x, y, z);
