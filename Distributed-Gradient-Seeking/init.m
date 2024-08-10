%% Initialization
close all;
clearvars;
clc;
% rng(0);

% Add the functions folder to the path
addpath(genpath(pwd));

% Set the default plotting parameters
set(0,'defaulttextinterpreter', 'latex');
set(groot, 'defaultAxesTickLabelInterpreter', 'latex');
set(groot, 'defaultLegendInterpreter', 'latex');

% Set the default font size
set(0,'defaultAxesFontSize', 20);
set(0,'DefaultLegendFontSize', 20);
set(0,'DefaultFigureWindowStyle','normal');
set(0,'DefaultUicontrolFontsize', 14);