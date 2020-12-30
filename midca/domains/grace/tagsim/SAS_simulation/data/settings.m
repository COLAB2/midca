function [param] = settings()

% System dynamics parameters 
param.Ixx = 1.2;
param.Iyy = 1.2;
param.Izz = 2.3;
param.k_quad = 1;
param.l = 0.25;
param.m = 2;
param.b = 0.2;
param.g = 9.81;


param.t0 = 0; % initial time
param.tf = 25; % final time %27

% Nominal system input
param.u_nominal = @(x) [param.m*param.g/(4*param.k_quad);param.m*param.g/(4*param.k_quad);param.m*param.g/(4*param.k_quad);param.m*param.g/(4*param.k_quad)]; % if u is a function of x


param.ulen = 4; % number of inputs

%Numerical tolerance
param.epsilon = 1e-5;

end

