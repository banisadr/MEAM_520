%% analyze_puma_starter.m
% 
% This Matlab script is part of the starter code for Homework 7 in MEAM 520
% at the University of Pennsylvania.


%% SETUP

% Clear all variables from the workspace.
clear all

% Home the console, so we can more easily find any errors that may occur.
home

% Set student names.
studentNames = 'PUT YOUR NAME HERE';


%% CALCULATE THE ARM'S SYMBOLIC LINEAR VELOCITY JACOBIAN

% Define real-valued symbolic variables for all six joint angles.
syms th1 th2 th3 th4 th5 th6 real

% Put your calculations here.

% For now, set Jv equal to a 3x3 zeros matrix.  You must change this.
Jv = zeros(3,3);


%% DETERMINE THE ARM'S LINEAR VELOCITY SINGULARITIES

% Put your calculations here.

% Display the simplified symbolic determinant of Jv on the command line.
% For now I am just setting this to be equal to th1 for demo purposes.
detJv = th1


%% PLOT THE ARM IN EACH ARM SINGULARITY

% Explain when the first singularity occurs.
disp('As shown in Figure 1, the PUMA arm''s first singularity occurs when...')

% Set the values of the joint angles for the first singularity.  Here I am
% just using the home configuration for demo purposes.
theta1 = 0;
theta2 = 0;
theta3 = 0;
theta4 = 0;
theta5 = -pi/2;
theta6 = 0;

% Calculate the linear velocity Jacobian for this pose. For now this is
% just set to identity. You must change this. 
Jvnum = eye(3);

% Set the angular velocity Jacobian for this pose to zero so we can focus on linear velocity. 
Jwnum = zeros(3);

% Open figure 1 and clear it.
figure(1)
clf

% Plot the robot in this first singularity.
plot_puma_starter(theta1, theta2, theta3, theta4, theta5, theta6, Jvnum, Jwnum, 'First Arm Singularity', studentNames);

% Explain when the second singularity occurs.
disp('As shown in Figure 2, the PUMA arm''s second singularity occurs when...')

% Set the values of the joint angles for the second singularity.  Here I am
% just using an arbitrary configuration for demo purposes.
theta1 = .2;
theta2 = pi/3;
theta3 = pi;
theta4 = .3;
theta5 = .9;
theta6 = -.2;

% Calculate the linear velocity Jacobian for this pose. For now this is
% just set to identity. You must change this. 
Jvnum = eye(3);

% Set the angular velocity Jacobian for this pose to zero so we can focus on linear velocity. 
Jwnum = zeros(3);

% Open figure 2 and clear it.
figure(2)
clf

% Plot the robot.
plot_puma_starter(theta1, theta2, theta3, theta4, theta5, theta6, Jvnum, Jwnum, 'Second Arm Singularity', studentNames);


%% CALCULATE THE WRIST'S SYMBOLIC ANGULAR VELOCITY JACOBIAN

% Put your calculations here.

% For now, set Jw equal to a 3x3 zeros matrix.  You must change this.
Jw = zeros(3,3);


%% DETERMINE THE WRIST'S ANGULAR VELOCITY SINGULARITIES

% Put your calculations here.

% Display the simplified symbolic determinant of Jw on the command line.
% For now I am just setting this to be equal to th2 for demo purposes.
detJw = th2


%% PLOT THE WRIST IN EACH SINGULARITY

% Explain when the first wrist singularity occurs.
disp('As shown in Figure 3, the PUMA wrist''s first singularity occurs when...')

% Set the values of the joint angles for the first singularity.  Here I am
% just using an arbitrary configuration for demo purposes.
theta1 = pi;
theta2 = 0;
theta3 = 0;
theta4 = pi;
theta5 = -pi/2;
theta6 = 0;

% Set the linear velocity Jacobian for this pose to zero so we can focus on angular velocity. 
Jvnum = zeros(3);

% Calculate the angular velocity Jacobian for this pose. For now this is
% just set to identity. You must change this. 
Jwnum = eye(3);

% Open figure 3 and clear it.
figure(3)
clf

% Plot the robot.
plot_puma_starter(theta1, theta2, theta3, theta4, theta5, theta6, Jvnum, Jwnum, 'First Wrist Singularity', studentNames);


%% PLOT THE ROBOT IN ANOTHER INTERESTING CONFIGURATION

% Set the values of the joint angles for the first singularity.  Here I am
% just using an arbitrary configuration for demo purposes.
theta1 = 0;
theta2 = 0;
theta3 = 0;
theta4 = 0;
theta5 = -pi/2;
theta6 = 0;

% Calculate linear and angular velocity Jacobians for this pose.
% For now these are just set to ones and zeros.  You should change this as needed.
Jvnum = ones(3);
Jwnum = zeros(3);

% Open figure 4 and clear it.
figure(4)
clf

% Plot the robot in this first singularity.  I am passing in 3x3 zero
% matrices for Jv and Jw because that part of the plotting routine is not
% yet needed.
plot_puma_starter(theta1, theta2, theta3, theta4, theta5, theta6, Jvnum, Jwnum, 'Arbitrary Configuration', studentNames);
