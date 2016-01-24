%% simulate_motor_weka_bahram.m
% 
% This Matlab script provides the main starter code for the motor
% simulation on Homework 8 in MEAM 520 at the University of Pennsylvania.
%
% The original was written by Professor Katherine J. Kuchenbecker. Students
% will modify this code to create their own script. Post questions on the
% class's Piazza forum. 
% 
% Change the name of this file to replace "starter" with your PennKey(s).


%% SETUP

% Clear all variables from the workspace.
clear all

% Home the console, so you can more easily find any errors that may occur.
home

% Input your names.
studentNames = 'Bahram Banisadr and Kate Wessels';


%% Parameters
% Set parameters of the system we want to simulate, noting units.
% Make them global so that the compute derivatives function can see them.
global Vperiod Vpulse
Vperiod = 0.5; % s, period of the pulse waveform being applied to the motor.
Vpulse = 12; % V, voltage of the pulse waveform being applied to the motor.


%% Time
% Define the simulation's start time, end time, and maximum time step.
tstart = 0;
tfinal = 1;
tstepmax = 0.001; % Maximum time step, in seconds.

% Set the time step for the graphical display to control playback speed.
graphical_tstep = 0.001; % s


%% Initial Conditions
% Define the initial conditions for the motor.
thetam0 = 0; % rad
omegam0 = 0; % rad/s
ia0 = 0; % A

% Put initial conditions into vector.
X0 = [thetam0; omegam0; ia0];


%% Simulation
% Show a message to explain how to cancel the graph.
disp('Click in this window and press control-c to cancel simulation or graphing if it is taking too long.')

% Run the simulation using ode45.
% The state equation function must be in the same directory as this script
% for Matlab to find it.  The @ makes the name a function handle, so ode45
% can call it over and over.  The other two inputs are the time vector and
% the initial condition5s.  The outputs are the resulting time vector (nx1)
% and the resulting state vector (nx3).
[t, Xhistory] = ode45(@compute_motor_derivatives_weka_bahram, [tstart:graphical_tstep:tfinal]', X0, odeset('MaxStep',tstepmax));


%% Pull out variables for plotting

thetam = Xhistory(:,1); % Motor angle in radians.
omegam = Xhistory(:,2); % Motor angular velocity in radians per second.
ia = Xhistory(:,3); % Motor current in amps.

% Calculate input voltage using the same equation used in the compute
% derivatives function.
V = Vpulse * (mod(t,Vperiod) > (Vperiod/2));


%% Graphing
% Plot the position, velocity, and current over time in figure 1.
figure(4)
subplot(3,1,1)
plot(t,thetam,'r-')
xlabel('Time (s)')
ylabel('Motor Position (rad)')
% Add a title including the student's names.
title(['Motor Simulation by ' studentNames]);

subplot(3,1,2)
plot(t,omegam,'b-')
xlabel('Time (s)')
ylabel('Motor Velocity (rad/s)')

subplot(3,1,3)
R = 2.18; % Ohm, from data sheet.
plot(t,V/R,'g--',t,ia,'g-')
legend('Desired','Actual')
xlabel('Time (s)')
ylabel('Motor Current (A)')
