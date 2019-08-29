function [] = example_vectorized_surf_plot
% File:      example_vectorized_surf_plot.m
% Author:    Ioannis Filippidis, jfilippidis@gmail.com
% Date:      2012.01.14 - 2012.02.11
% Language:  MATLAB R2011b
% Purpose:   test meshgrid interface to functions taking as argument a
%            matrix of column vectors
% Copyright: Ioannis Filippidis, 2012-

dom = [-2, 2, -2, 2];
res = [31, 30];

[X, Y] = domain2meshgrid(dom, res);
q = meshgrid2vec(X, Y);
f = testf(q);
f = vec2meshgrid(f, X);

surf(X, Y, f)

function [f] = testf(q)
q0 = [1, 2].';
q_q0 = bsxfun(@minus, q, q0);
f = sqrt(sum(q_q0.^2, 1) ); % instead use vnorm (Mathworks File Exchange)
f = cos(f) .*sin(f);
