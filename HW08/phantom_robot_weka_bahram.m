%% phantom_robot_weka_bahram.m
% 
% This Matlab script provides the main starter code for the Phantom robot
% on Homework 8 in MEAM 520 at the University of Pennsylvania.
%
% The original was written by Professor Katherine J. Kuchenbecker. Students
% will modify this code to create their own script. Post questions on the
% class's Piazza forum. 



%% SETUP

% Clear all variables from the workspace.
clear all

% Home the console, so you can more easily find any errors that may occur.
home

% Input your names and PennKeys as strings.
studentNames = 'Kate Wessels and Bahram Banisadr';
yourpennkeys = 'weka_bahram'; % Replace starter with your PennKey or PennKeys.

% Create function names for four main functions bassed on your PennKeys.  
f1 = ['@phantom_counts_to_angles_' yourpennkeys];
phantom_counts_to_angles = eval(f1);
f2 = ['@phantom_angles_to_positions_' yourpennkeys];
phantom_angles_to_positions = eval(f2);
f3 = ['@phantom_force_to_torques_' yourpennkeys];
phantom_force_to_torques = eval(f3);
f4 = ['@phantom_torques_to_voltages_' yourpennkeys];
phantom_torques_to_voltages = eval(f4);

% Set whether to animate the robot's movement and how much to slow it down.
pause on; % Set this to off if you don't want to watch the animation.
GraphingTimeDelay = 0.001; % The length of time that Matlab should pause between positions when graphing, if at all, in seconds.

% Set the amount by which to scale forces for the plot.
force_scale = 50; % mm/N


%% LOAD DATA
% This problem is about the SensAble Phantom Premium 1.0, a three-dof
% impedance-type haptic interface.  Load data recorded from it.
filename = 'phantom_encoder_counts.txt';
Q123 = load(filename,'-ascii');
Q1_history = Q123(:,1);
Q2_history = Q123(:,2);
Q3_history = Q123(:,3);


%% DEFINE FORCE OUTPUT
% Define the force vector to output, in newtons, in frame 0, at tip.
Fx = 3; % N
Fy = 1; % N
Fz = 2; % N


%% SIMULATION

% Notify the user that we're starting the animation.
disp('Starting the animation.')

% Show a message to explain how to cancel the simulation and graphing.
disp('Click in this window and press control-c to stop the code.')

% Initialize matrices to keep history of robot movement over time.
theta1_history = zeros(1,length(Q123));
theta2_history = zeros(1,length(Q123));
theta3_history = zeros(1,length(Q123));
tipx_history = zeros(1,length(Q123));
tipy_history = zeros(1,length(Q123));
tipz_history = zeros(1,length(Q123));
Fx_history = zeros(1,length(Q123));
Fy_history = zeros(1,length(Q123));
Fz_history = zeros(1,length(Q123));
tau1_history = zeros(1,length(Q123));
tau2_history = zeros(1,length(Q123));
tau3_history = zeros(1,length(Q123));
V1_history = zeros(1,length(Q123));
V2_history = zeros(1,length(Q123));
V3_history = zeros(1,length(Q123));


% Step through the time vector to animate the robot.
for i = 1:length(Q123)
    
    % Pull the current values of the joint angles from their histories.
    Q1 = Q1_history(i);
    Q2 = Q2_history(i);
    Q3 = Q3_history(i);

    % Convert encoder counts to joint angles in radians.
    [theta1, theta2, theta3] = phantom_counts_to_angles(Q1, Q2, Q3);
    
    % Convert joint angles to robot positions (four or more points along robot arm).
    [p1, p2, p3, p4] = phantom_angles_to_positions(theta1, theta2, theta3);
    
    % Put the points together for plotting.
    points_to_plot = [p1 p2 p3 p4];
    
    % Calculate joint torques needed to exert the desired force, given
    % current robot configuration.
    [tau1, tau2, tau3] = phantom_force_to_torques(Fx, Fy, Fz, theta1, theta2, theta3);
    
    % Convert joint torques to DAC voltages.
    [V1, V2, V3] = phantom_torques_to_voltages(tau1, tau2, tau3);
    
    % Store calculated values in arrays for plotting after animation.
    theta1_history(i) = theta1;
    theta2_history(i) = theta2;
    theta3_history(i) = theta3;    
    tipx_history(i) = points_to_plot(1,end);
    tipy_history(i) = points_to_plot(2,end);
    tipz_history(i) = points_to_plot(3,end);
    Fx_history(i) = Fx;
    Fy_history(i) = Fy;
    Fz_history(i) = Fz;
    tau1_history(i) = tau1;
    tau2_history(i) = tau2;
    tau3_history(i) = tau3;
    V1_history(i) = V1;
    V2_history(i) = V2;
    V3_history(i) = V3;
    
    % Check if this is the first time we are plotting.
    if (i == 1)
        % Open figure 1.
        figure(1); clf;
        
        % The first time, plot the robot points and keep a handle to the plot.
        % This is a 3D plot with dots at the points and lines connecting
        % neighboring points, made thicker, with big dots, in black.
        hrobot = plot3(points_to_plot(1,:), points_to_plot(2,:), points_to_plot(3,:), 'k.-', 'linewidth', 6, 'markersize', 25);
        
        % Also plot the tip position of the robot and the force vector,
        % using hold on and hold off, also keeping a handle to the plot so
        % we can update the data points later.
        hold on;
        htip = plot3(tipx_history(i), tipy_history(i), tipz_history(i), '-', 'color',[0 .7 0], 'linewidth', 2);
        hforce = plot3(tipx_history(i) + force_scale*[0 Fx], tipy_history(i) + force_scale*[0 Fy], tipz_history(i) + force_scale*[0 Fz], '-', 'color',[.5 .5 .5],'linewidth',2);
        hold off;

        % Label the axes.
        xlabel('X (mm)');
        ylabel('Y (mm)');
        zlabel('Z (mm)');

        % Turn on the grid and the box.
        grid on;
        box on;

        % Set the view to see the robot easily.
        view(42, 30)

        % Set the axis properties for 3D visualization with one unit the
        % same in every direction. 
        axis equal vis3d

        % Set the axis limits.
        axis([0 250 -125 125 0 250])

        % Put text on the plot to show how much time has elapsed.  The text
        % is centered.
        htime = text(50,-50,10,sprintf('i = %d',i),'horizontalAlignment','center');

        % Add a title including the student's names.
        title(['Phantom Robot by ' studentNames]);
    else
        % Once the animation has been set up, we don't need to reformat the
        % whole plot.  We just set the data to the correct new values for
        % the robot animation, the tip history, the force, and the text
        % showing the elapsed time.
        set(hrobot, 'xdata', points_to_plot(1,:), 'ydata', points_to_plot(2,:), 'zdata', points_to_plot(3,:))
        set(htip, 'xdata', tipx_history(1:i), 'ydata', tipy_history(1:i), 'zdata', tipz_history(1:i))
        set(hforce, 'xdata', tipx_history(i) + force_scale*[0 Fx], 'ydata', tipy_history(i) + force_scale*[0 Fy], 'zdata', tipz_history(i) + force_scale*[0 Fz]);
        set(htime, 'string', (sprintf('i = %d',i)));
    end
    
    % Pause for a short duration so that the viewer can watch animation evolve over time.
    pause(GraphingTimeDelay)
    
end

disp('Done with the animation.')


%% ADDITIONAL PLOTS

i = 1:length(Q123);

% FIGURE 2 - encoders, joint angles, and positions
figure(2)
subplot(3,1,1)
plot(i,Q123);
xlabel('Sample Number')
ylabel('Encoder Value (counts)')
legend('Q_1', 'Q_2', 'Q_3')

subplot(3,1,2)
plot(i,[theta1_history' theta2_history' theta3_history'])
xlabel('Sample Number')
ylabel('Joint Angle (radians)')
legend('\theta_1', '\theta_2', '\theta_3')

subplot(3,1,3)
h = plot(i,[tipx_history' tipy_history' tipz_history']);
set(h(1),'color',[0 .7 .7])
set(h(2),'color',[.7 .7 0])
set(h(3),'color',[.7 0 .7])
xlabel('Sample Number')
ylabel('Tip Position (mm)')
legend('x', 'y', 'z')

% FIGURE 3 - forces, joint torques, and voltages
figure(3)
subplot(3,1,1)
h = plot(i,[Fx_history' Fy_history' Fz_history']);
set(h(1),'color',[0 .7 .7])
set(h(2),'color',[.7 .7 0])
set(h(3),'color',[.7 0 .7])
xlabel('Sample Number')
ylabel('Force (N)')
legend('F_x', 'F_y', 'F_z')

subplot(3,1,2)
plot(i,[tau1_history' tau2_history' tau3_history'])
xlabel('Sample Number')
ylabel('Joint Torque (Nm)')
legend('\tau_1', '\tau_2', '\tau_3')

subplot(3,1,3)
plot(i,[V1_history' V2_history' V3_history'])
xlabel('Sample Number')
ylabel('DAC Voltage (V)')
legend('V_1', 'V_2', 'V_3')

% Select figure 1.
figure(1)
