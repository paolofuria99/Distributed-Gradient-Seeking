clear all;close all;clc;
% Create a bivariate standard normal.
% First create a grid for the domain.
[x,y] = meshgrid(-50:.5:50,-50:.5:50);
% Evaluate using the bivariate standard normal.
%z = (1/(2*pi))*exp(-0.5*(x.^2+y.^2));
z=10*exp(-((x-10).^2 + (y-8).^2)/600)+18*exp(-((x+20).^2+(y+12).^2)/200);
% Do the plot as a surface.
surf(x,y,z)

% Create a 2-D contour plot with labels.
% This returns the information for the labels.
 c = contourf(x,y,z,100);
% % % Add the labels to the plot.
clabel(c)