function plot_puma_starter(theta1, theta2, theta3, theta4, theta5, theta6, Jv, Jw, description, studentNames)
% This Matlab function is part of the starter code for Homework 7 in MEAM
% 520 at the University of Pennsylvania.  It plots the PUMA robot in the
% specified configuration, annotated with the current linear velocity and
% angular velocity manipulability ellipsoids.



%% ROBOT PARAMETERS

% This problem is about the PUMA 260 robot, a 6-DOF manipulator.

% Define the robot's measurements.  These correspond to the diagram in the
% homework and are constant.
a = 13.0; % inches
b =  2.5; % inches
c =  8.0; % inches
d =  2.5; % inches
e =  8.0; % inches
f =  2.5; % inches

% Length of coordinate frame vectors, in inches.
vlen = 8;


%% DH MATRICES

% Calculate the six A matrices using the provided DH function.
A1 = dh_kuchenbe(0,  pi/2,   a, theta1);
A2 = dh_kuchenbe(c,     0,  -b, theta2);
A3 = dh_kuchenbe(0, -pi/2,  -d, theta3);
A4 = dh_kuchenbe(0,  pi/2,   e, theta4);
A5 = dh_kuchenbe(0, -pi/2,   0, theta5);
A6 = dh_kuchenbe(0,     0,   f, theta6);


%% ORIGIN POSITIONS

% Define the homogeneous representation of the origin of any frame.
o = [0 0 0 1]';

% Calculate the position of the origin of each frame in the base frame.
o0 = o;
o1 = A1*o;
extra_point = A1*[0 0 -b 1]';
o2 = A1*A2*o;
o3 = A1*A2*A3*o;
o4 = A1*A2*A3*A4*o;
o5 = A1*A2*A3*A4*A5*o;
o6 = A1*A2*A3*A4*A5*A6*o;

% Put the seven origin points together for plotting.
points_to_plot = [o0 o1 extra_point o2 o3 o4 o5 o6];


%% END-EFFECTOR COORDINATE FRAME 

% Save the full transformation in T06.
T06 = A1*A2*A3*A4*A5*A6;

% Calculate the coordinates of the x, y, and z unit vectors of frame 6 in
% frame 0. Each of these vectors starts at the origin of frame 6 and ends
% at the distance vlen along the designated axis.  We calculate the
% location of the end by multiplying T06 into a scaled unit vector in the
% correct direction.
x06 = [o6 (T06 * [vlen 0 0 1]')];
y06 = [o6 (T06 * [0 vlen 0 1]')];
z06 = [o6 (T06 * [0 0 vlen 1]')];


%% PLOT ROBOT

% Plot the robot in 3D using big dots at the points and thick lines
% to connect neighboring points.
plot3(points_to_plot(1,:), points_to_plot(2,:), points_to_plot(3,:), ...
    '.-','linewidth',7,'markersize',35,'color',0.5*[1 .88 .75])

% Call hold on so we can plot more things on this axis.
hold on

% Plot the x, y, and z axes of frame 6.
plot3(x06(1,:), x06(2,:), x06(3,:), 'k:', 'linewidth',2)
plot3(y06(1,:), y06(2,:), y06(3,:), 'k--', 'linewidth',2)
plot3(z06(1,:), z06(2,:), z06(3,:), 'k-', 'linewidth',2)

% Label the axes.
xlabel('X (in.)')
ylabel('Y (in.)')
zlabel('Z (in.)')

% Turn on the box.
box on

% Turn on the grid (if you want) by uncommenting the following line.
%grid on;

% Set the axis properties to make one unit the same in every
% direction and enable 3D rotation. 
axis equal vis3d

% Set the axis limits.
axis([-20 20 -20 20 0 40])

% Set the viewing angle.
view(80,20)

% Add a title that includes the names of the students doing this homework.
title(['PUMA: ' description ' by ' studentNames])


%% CREATE SPHERE - FOR DEMONSTRATION

% Set the number of segments.
n = 21;

% Create theta as a column vector of n angles going between 0 and 2*pi rad.
theta = linspace(0,2*pi,n);

% Create phi as a row vector of n angles going between -pi/2 and pi/2 rad.
phi = linspace(-pi/2,pi/2,n)';
 
% Create a set of unit vectors from the angles theta and phi.  Each of the
% results ux, uy, and uz is an n x n matrix holding coordinates of a point
% on the sphere.  This is the format needed to use the surf command.
ux = cos(phi) * cos(theta);
uy = cos(phi) * sin(theta);
uz = sin(phi) * ones(1,length(theta));


%% PLOT SPHERE - FOR DEMONSTRATION

% Define an arbitrary center location, in inches.
cx = 0;
cy = 0;
cz = 20;

% Define an arbitrary sphere radius, in inches.
r = 5;

% Plot a scaled sphere at this location.  Make its edges and faces
% transparent so you can still see the robot.
surf(cx + r*ux, cy + r*uy, cz + r*uz,'FaceAlpha',.4,'EdgeAlpha',.4)

% Set the colormap to something fun.  
colormap hot

% Call hold off because we are done plotting things.
hold off

