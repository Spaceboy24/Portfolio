%% Issues:
% Lack of accuracy to real world problems in Nominal Trajectory
% Causes initial guesses to be off
% End point continuity constraints
% Too many constraints
% For every piecewise polynomials in horizon, n*(2*(k-1)) constraints 
% k is the number of piecewise polys and n is the number of states

%%
close all;
clear all;
clc;

x_n = linspace(0,pi,51);
y_n = sin(x_n);
plot(x_n,y_n), hold on
% x_nom = [0.0000 0.9132 1.8256 2.7372 3.6480 4.5580; 4.5580 5.4681 6.3725 7.2543 8.1478 8.4766;
%         8.4766 8.8054 9.1304 9.4585 9.7757 10.0000];
% y_nom = [0.0000 0.5833 1.1679 1.7537 2.3408 2.9291; 2.9291 3.5174 4.1038 4.6769 5.2604 5.4753;
%         5.4753 6.6902 7.1199 7.9090 9.0990 10.0000];
% t_nom = 3;

[x_nom, y_nom, t_nom] = get_init()

tic
for l = 1:5
   
    [x_opt, y_opt, t_opt] = RH_optimization(x_nom, y_nom, t_nom);
    
    x_nom = x_opt;
    y_nom = y_opt;
    t_nom = t_opt;
  
end
toc