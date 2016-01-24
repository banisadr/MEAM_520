%% analyze_puma_bahram.m
% 
% This Matlab script is part of the starter code for Homework 7 in MEAM 520
% at the University of Pennsylvania.


%% SETUP

% Clear all variables from the workspace.
clear all

% Home the console, so we can more easily find any errors that may occur.
home

% Set student names.
studentNames = 'Bahram Banisadr';


%% CALCULATE THE ARM'S SYMBOLIC LINEAR VELOCITY JACOBIAN

% Define real-valued symbolic variables for all six joint angles.
syms th1 th2 th3 th4 th5 th6 real
syms a b c d e f

% Put your calculations here.

o = [0 0 0 1]';

A1 = dh_kuchenbe(0,  pi/2,   a, th1);
A2 = dh_kuchenbe(c,     0,  -b, th2);
A3 = dh_kuchenbe(0, -pi/2,  -d, th3);
A4 = dh_kuchenbe(0,  pi/2,   e, th4);
A5 = dh_kuchenbe(0, -pi/2,   0, th5);
A6 = dh_kuchenbe(0,     0,   f, th6);

o4 = simplify(A1*A2*A3*A4*o);


% For now, set Jv equal to a 3x3 zeros matrix.  You must change this.
Jv = [(diff(o4(1),th1)) (diff(o4(1),th2)) (diff(o4(1),th3));
     (diff(o4(2),th1)) (diff(o4(2),th2)) (diff(o4(2),th3));
     (diff(o4(3),th1)) (diff(o4(3),th2)) (diff(o4(3),th3))];
    

%% DETERMINE THE ARM'S LINEAR VELOCITY SINGULARITIES

% det(Jv) = 512*cos(th3)*(sin(th2 + th3) - cos(th2))
% cos(th3)*(sin(th2 + th3) - cos(th2)) = 0
% 
% Singularity 1: cos(th3) = 0
% th3 = pi/2, -pi/2
% 
% Singularity 2: sin(th2+th3) = cos(th2)
% th3 = pi/2

% Display the simplified symbolic determinant of Jv on the command line.
% For now I am just setting this to be equal to th1 for demo purposes.
detJv = simplify(det(Jv))


%% PLOT THE ARM IN EACH ARM SINGULARITY

% Explain when the first singularity occurs.
disp('As shown in Figure 1, the PUMA arm''s first singularity occurs when joint three is at an angle such that link c and e are parallel (-pi/2, pi/2)')

% Set the values of the joint angles for the first singularity.
theta1 = 0;
theta2 = 0;
theta3 = -pi/2;
theta4 = 0;
theta5 = -pi/2;
theta6 = 0;

a = 13.0; % inches
b =  2.5; % inches
c =  8.0; % inches
d =  2.5; % inches
e =  8.0; % inches
f =  2.5; % inches

% Calculate the linear velocity Jacobian for this pose.

th1 = theta1;
th2 = theta2;
th3 = theta3;

Jvnum = eval(Jv);

% Set the angular velocity Jacobian for this pose to zero so we can focus on linear velocity. 
Jwnum = zeros(3);

% Open figure 1 and clear it.
figure(1)
clf

% Plot the robot in this first singularity.
plot_puma_bahram(theta1, theta2, theta3, theta4, theta5, theta6, Jvnum, Jwnum, 'First Arm Singularity', studentNames);

% Explain when the second singularity occurs.
disp('As shown in Figure 2, the PUMA arm''s second singularity occurs when joint three is at pi/2 and therefore link e is parrallel to link c and pointed straight backward')

% Set the values of the joint angles for the second singularity.

theta1 = .2;
theta2 = pi/3;
theta3 = pi/2;
theta4 = .3;
theta5 = .9;
theta6 = -.2;

% Calculate the linear velocity Jacobian for this pose.

th1 = theta1;
th2 = theta2;
th3 = theta3;

Jvnum = eval(Jv);

% Set the angular velocity Jacobian for this pose to zero so we can focus on linear velocity. 
Jwnum = zeros(3);

% Open figure 2 and clear it.
figure(2)
clf

% Plot the robot.
plot_puma_bahram(theta1, theta2, theta3, theta4, theta5, theta6, Jvnum, Jwnum, 'Second Arm Singularity', studentNames);


%% CALCULATE THE WRIST'S SYMBOLIC ANGULAR VELOCITY JACOBIAN

T_3_0 = A1*A2*A3;
T_4_0 = A1*A2*A3*A4;
T_5_0 = A1*A2*A3*A4*A5;

k = [0 0 1]';

z3 = simplify(T_3_0(1:3,1:3)*k);
z4 = simplify(T_4_0(1:3,1:3)*k);
z5 = simplify(T_5_0(1:3,1:3)*k);

% For now, set Jw equal to a 3x3 zeros matrix.  You must change this.
Jw = [z3 z4 z5];


%% DETERMINE THE WRIST'S ANGULAR VELOCITY SINGULARITIES

% det(Jw) = -sin(th5) = 0
% 
% Singularity: th5 = 0, pi

% Display the simplified symbolic determinant of Jw on the command line

detJw = -sin(th5)


%% PLOT THE WRIST IN EACH SINGULARITY

% Explain when the first wrist singularity occurs.
disp('As shown in Figure 3, the PUMA wrist''s first singularity occurs when joint 5 is at 0 and joints 4 and 6 are therefore alligned and redundant')

% Set the values of the joint angles for the first singularity.
theta1 = pi;
theta2 = 0;
theta3 = 0;
theta4 = pi;
theta5 = 0;
theta6 = 0;

% Set the linear velocity Jacobian for this pose to zero so we can focus on angular velocity. 
Jvnum = zeros(3);

% Calculate the angular velocity Jacobian for this pose.

th1 = theta1;
th2 = theta2;
th3 = theta3;
th4 = theta4;
th5 = theta5;
th6 = theta6;

Jwnum = eval(Jw);

% Open figure 3 and clear it.
figure(3)
clf

% Plot the robot.
plot_puma_bahram(theta1, theta2, theta3, theta4, theta5, theta6, Jvnum, Jwnum, 'First Wrist Singularity', studentNames);


%% PLOT THE ROBOT IN ANOTHER INTERESTING CONFIGURATION

% Set the values of the joint angles to an arbitrary configuration
theta1 = pi/3;
theta2 = pi/4;
theta3 = -pi/4;
theta4 = 0;
theta5 = pi/2;
theta6 = 0;

% Calculate linear and angular velocity Jacobians for this pose.
Jvnum = zeros(3);
Jwnum = eval(Jw);

% Open figure 4 and clear it.
figure(4)
clf

% Plot the robot in this first singularity.  I am passing in 3x3 zero
% matrices for Jv and Jw because that part of the plotting routine is not
% yet needed.
plot_puma_bahram(theta1, theta2, theta3, theta4, theta5, theta6, Jvnum, Jwnum, 'Arbitrary Configuration', studentNames);
