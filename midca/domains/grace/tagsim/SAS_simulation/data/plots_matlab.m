%clear all

global param 
global master

%Read and format states and time
states = csvread('states.csv');
time = states(:,1);
states(:,1) = [];
%Read and format actions
% master.all_actions = csvread('actions.csv');
% master.all_actions(:,end) = [];

% Load all settings in struct "param" 
param = settings();


figure, plot(states(:,1),states(:,2))
axis equal
axis([0,1,0,1])

figure(2), plot(time,states(:,3))

%Plots
%plot(time,states(:,1:6))
%legend('x','y','z','phi','theta','psi')

% plot_control();