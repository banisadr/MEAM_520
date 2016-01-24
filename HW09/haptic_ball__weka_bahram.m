%% haptic_ball_starter.m
%
% This script enables the user to touch a virtual ball using a PHANToM
% Premium 1.0 haptic interface.  The user's position is shown as a small
% red circle, and ball's rainbow surfaces are transparent.  The user is
% outside the ball, and the ball should be rendered using a virtual spring.
%
% Written by Katherine J. Kuchenbecker for MEAM 520 at the University of Pennsylvania.
% 
% Your team must complete this script.  Change the filename to replace
% "starter" with your two PennKeys (i.e., "pennkey1_pennkey2"), and write
% the names of your team members below. 


%% Clean up

clear all
close all
clc

% Input your names as a string.
studentNames = 'Kate Wessels and Bahram Banisadr';


%% Set hardware mode, duration, and warnings

% Set whether to use the actual PHANToM hardware.  If this variable is set
% to false, the software will simulate the presence of a user by reading a
% pre-recorded trajectory.  You should use this mode to debug your code
% before running anything on the PHANToM computer.  Once you are on the
% PHANToM/PUMA computer in Towne B2, you may set this variable to true.  
% Always first run your code with the emergency stop down to prevent the
% application of forces and make sure everything works correctly on the real
% robot.  Hold onto the PHANToM tightly and keep a hand on the emergency stop.
hardware = false;

% Set how many times we want our servo loop to run.  Each cycle takes about
% two milliseconds on the computer in Towne B2.  There are 10000 cycles in
% the recorded data file.  If nCycles exceeds 10000, the motion loops.
nCycles = 2000;

% If not using the hardware, turn off warnings triggered when you command
% joint torques above the limits. 
if (~hardware)
    warning('off','PHANToM:JointTorque')
else
    warning('on','PHANToM:JointTorque')
end


%% Define the virtual environment

% Set the stiffness of the virtual walls in newtons per millimeter.
k = 0.05; % Keep this value positive and smaller than 0.5

% The location of the center of the ball, relative to the origin of the
% Phantom's coordinate frame.  X is positive toward the user, Y is positive
% to the right, and Z is positive up. All dimensions are in millimeters.
ballCenterX = 150;
ballCenterY = 0;
ballCenterZ = 175;

% Set the radius of the ball in millimeters.
ballRadius = 50;

% Set the alpha (transparency) of the ball's surfaces.
ballFaceAlpha = 0.5;

% Set the scale of the force vector in millimeters per newton.
fScale = 40;


%% Set up the figure for graphing

% Open figure 1.
figure(1);

% Plot a small red circle to represent the position of the PHANToM.  For
% now, we just put it at the origin.  We keep the handle to this plot
% (hPhantomDot) so that we can move the dot later.
hPhantomDot = plot3(0, 0, 0,'ro');

% Turn hold on to let us put more things on the graph.
hold on

% Plot a thick black line to represent the force vector.  For now, we just
% have it go from the origin to [10 10 10]; later we will set it to be a
% scaled version of the commanded force vector.  We keep the handle to this
% plot (hForceLine) so we can move the line later.
hForceLine = plot3([0 10], [0 10], [0 10],'k-','linewidth',2);

% Set the axis limits, the viewing direction, and other properties of
% the view.
axis equal vis3d
axis([0 300 -150 150 0 300]);
set(gca,'Xtick',0:50:300,'ytick',-150:50:150,'ztick',0:50:300,'color',[.4 .4 .4])
view(100,16);
colormap hot
box on
grid on

% Label the axes.
xlabel('X (mm)')
ylabel('Y (mm)')
zlabel('Z (mm)')

% Add a title.
title(['Haptic Ball by ' studentNames])


%% Draw the ball

% Get the coordinates of the points on a unit sphere with resolution 20.
[xsphere, ysphere, zsphere] = sphere(20);

% Plot the ball as a surface at its center location, with its radius.
hBall = surf(ballRadius*xsphere+ballCenterX, ballRadius*ysphere+ballCenterY, ballRadius*zsphere+ballCenterZ);

% Set the transparency of the ball's faces and edges.
set(hBall,'facealpha',ballFaceAlpha,'edgeAlpha',0)

% Turn hold off to stop allowing more graphing.
hold off;


%% Define the PHANToM parameters

% Link lengths in millimeters.
l1 = 168;
l2 = 140;
l3 = 140;


%% Do final preparations

% Start the Phantom, passing in hardware (true or false)
phantomStart(hardware);

% Initialize a vector to hold the time stamp for each cycle.
t = zeros(nCycles,1);

% Initialize vectors to store tip position and force output for each cycle.
hx_history = zeros(nCycles,1);
hy_history = zeros(nCycles,1);
hz_history = zeros(nCycles,1);
Fx_history = zeros(nCycles,1);
Fy_history = zeros(nCycles,1);
Fz_history = zeros(nCycles,1);

% Initialize a variable to hold the last time the graphics update was run.
lastGraphicsTime = 0;

% Call tic to latch the time.
tic;


%% Run the servo loop
for i = 1:nCycles
    % Measure the time and store it in our vector of time stamps.  
    % The units are seconds.
    t(i) = toc;
    
    % Get the Phantom's joint angles in radians.
    theta123 = phantomJointAngles;
    
    % Pull the three joint angles out of the vector to simplify calculations.
    theta1 = theta123(1);
    theta2 = theta123(2);
    theta3 = theta123(3);
    
    % Calculate the position of the haptic device's tip using forward kinematics.
    % We store the result in hx, hy, and hz.  The units are millimeters.
    r = l2*cos(theta2) - l3*sin(theta3);
    hx = r*cos(theta1);
    hy = -r*sin(theta1);
    hz = l1 - l2*sin(theta2) - l3*cos(theta3);
    
    % All your commands should be below this line.  Use hx, hy, hz, and
    % whatever else you need to calculate Fx, Fy, and Fz.
   % *********************************************************************
   
   
   r_xyz = sqrt((hx-ballCenterX)^2+(hz-ballCenterZ)^2+(hy-ballCenterY)^2);
   theta = acos((hz-ballCenterZ)/r_xyz);
   psi = atan2((hy-ballCenterY),(hx-ballCenterX));
   
   if r_xyz <= ballRadius
       Fr = k* (ballRadius - r_xyz);
       Fx = Fr*sin(theta)*cos(psi);
       Fy = Fr*sin(theta)*sin(psi);
       Fz = Fr*cos(theta);
   else
       Fx = 0;
       Fy = 0;
       Fz = 0;
   end
   
    
    % *********************************************************************
    % All your commands should be above this line, and Fx, Fy, and Fz
    % should be set to the force values you want the user to feel.

    % Calculate the linear velocity Jacobian for the PHANToM at the present
    % joint angles. This formula was derived by calculating the partial
    % derivatives of the x, y, and z tip position equations.
    Jv = [-sin(theta1)*(l2*cos(theta2)-l3*sin(theta3)) -l2*cos(theta1)*sin(theta2)  -l3*cos(theta1)*cos(theta3);
           -cos(theta1)*(l2*cos(theta2)-l3*sin(theta3))  l2*sin(theta1)*sin(theta2)   l3*sin(theta1)*cos(theta3);
                                                      0             -l2*cos(theta2)               l3*sin(theta3)];
                                                  
    % Calculate the three joint torques by multiplying the desired force
    % vector by the transpose of the Jacobian and dividing by 1000 (to
    % convert from millimeters to meters).  Units are newton-meters.
    tau123 = Jv' * [Fx Fy Fz]' / 1000;
    
    % Command the necessary joint torques.
    % If you're getting a lot of warnings for asking for too high of joint
    % torques, you can turn that warning off by uncommenting a line near
    % the top of this script.
    phantomJointTorques(tau123(1), tau123(2), tau123(3));
    
    % Store this cycle's values in the history vectors.
    hx_history(i) = hx;
    hy_history(i) = hy;
    hz_history(i) = hz;
    Fx_history(i) = Fx;
    Fy_history(i) = Fy;
    Fz_history(i) = Fz;
    
    % Check how much time has elapsed since we last updated the graphics.
    if (t(i) - lastGraphicsTime > 0.03)
        % Enough time has passed.
        
        % Update the graph by setting the data for the PHANToM's dot to the
        % position of the haptic device.
        set(hPhantomDot, 'xdata', hx, 'ydata', hy, 'zdata', hz)

        % Update the graph by setting the data for the force line to show a
        % scaled version of the commanded force.
        set(hForceLine,'xdata',[hx hx+Fx*fScale],'ydata',[hy hy+Fy*fScale], 'zdata',[hz hz+Fz*fScale]);

        % Store this time for future comparisons.
        lastGraphicsTime = t(i);
    end
    
    % Pause for one millisecond to keep things moving at a reasonable pace.
    pause(.001);
end

% Stop the Phantom.
phantomStop;


%% Plot the results

% Open figure 2 and clear it.
figure(2)
clf

% Plot positions over time in the top subplot.
subplot(2,1,1)
h = plot(t,[hx_history hy_history hz_history]);
set(h(1),'color',[0 .7 .7])
set(h(2),'color',[.7 .7 0])
set(h(3),'color',[.7 0 .7])
xlabel('Time (s)')
ylabel('Tip Position (mm)')
legend('h_x', 'h_y', 'h_z')

% Plot forces over time in the bottom subplot.
subplot(2,1,2)
h = plot(t,[Fx_history Fy_history Fz_history]);
set(h(1),'color',[0 .7 .7])
set(h(2),'color',[.7 .7 0])
set(h(3),'color',[.7 0 .7])
xlabel('Time (s)')
ylabel('Force (N)')
legend('F_x', 'F_y', 'F_z')
