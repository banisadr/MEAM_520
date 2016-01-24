%This assignment was done by Valerie Cohen and Bahram Banisadr for MEAM520, the introduction to
%robotics class at Penn, taught by professor Kuchenbecker. Some of
%professor Kuchenbecker's matlab functions and scripts, including
%dh_kuchenbe and scara_robot_kuchenbe, were referred to or used in part in
%this script.

clear
clc
studentNames = 'Valerie Cohen and Bahram Banisadr';
load puma_motion8
%all below variables in inches
a=13; %in
b=2.5;
c=8;
d=2.5;
e=8.0;
f=2.5;
% Define our time vector.
tStep = 0.04; % The simulation's time step, in seconds.
t = t_history;  % The time vector (a column vector).

% Set whether to animate the robot's movement and how much to slow it down.
pause on;  % Set this to off if you don't want to watch the animation.
GraphingTimeDelay = 0.001; % The length of time that Matlab should pause between positions when graphing, if at all, in seconds.

%link 1:
for i = 1:length(t_history)
    theta1 = theta1_history(i);
    theta2 = theta2_history(i);
    theta3 = theta3_history(i);
    theta4 = theta4_history(i);
    theta5 = theta5_history(i);
    theta6 = theta6_history(i);
    
    
    A1 = dh_kuchenbe(0,(pi/2),a,theta1);
    A2 = dh_kuchenbe(c,0,-b,theta2);
    A3 = dh_kuchenbe(0,(-pi/2),-d,theta3);
    A4 = dh_kuchenbe(0,(pi/2),e,theta4);
    A5 = dh_kuchenbe(0,(-pi/2),0,theta5);
    A6 = dh_kuchenbe(0,0,f,theta6);
    
    % Create the homogeneous representation of the origin of a frame.
    o = [0 0 0 1]';
    
    % Calculate the position of the origin of frame 0 expressed in frame 0.
    % By definition, this is just o.
    o0 = o;
    
    % Calculate the position of the origin of frame 1 expressed in frame 0.
    % We multiply A1 into the origin vector to find the position o1.
    o1 = A1*o;
    
    % Calculate the position of the origin of frame 2 expressed in frame 0.
    % We multiply A1 and A2 into the origin vector to find the position o2.
    o2 = A1*A2*o;
    
    % Calculate the position of the origin of frame 3 expressed in frame 0.
    % We multiply A1, A2, and A3 into the origin vector to find the position o3.
    o3 = A1*A2*A3*o;
    o4 = A1*A2*A3*A4*o;
    o5 = A1*A2*A3*A4*A5*o;
    o6 = A1*A2*A3*A4*A5*A6*o;
    
    T06 = A1*A2*A3*A4*A5*A6;
    T06_history(:,i) = T06(1:4,end);
    
    % Put the points together.  Each column is the homogeneous
    % representation of a point in the robot.  I have added an extra point
    % at [0 0 -2 1]' to give the visual illusion of a base; this is not
    % needed.  The points should go in order along the robot.
    points_to_plot = [[0 0 -2 1]' o0 o1 o2 o3 o4 o5 o6];
    % Grab the final plotted point for the trajectory graph.
    tip_history(:,i) = points_to_plot(1:3,end);
    
    % Open figure 1.
    figure(1);
    %this section was where we tried to do the end effector position and
    %orientation, but we couldnt figure it out and didnt have enough time.
    %{
    xend = points_to_plot(1,end);
    yend = points_to_plot(2,end);
    zend = points_to_plot(3,end);
    
    xend2 = T06*[1 0 0 1]';
    yend2 = T06*[0 1 0 1]';
    zend2 = T06*[0 0 1 1]';
    %}
    
    % Check if this is the first time we are plotting.
    if (i == 1)
        % The first time, plot the robot points and keep a handle to the plot.
        % This is a 3D plot with dots at the points and lines connecting
        % neighboring points, made thicker, with big dots, in dark gray.
        
        
        hrobot = plot3(points_to_plot(1,:),points_to_plot(2,:),points_to_plot(3,:),'.-','linewidth',5,'markersize',20,'color',[.3 .3 .3]);
        %endeffector_x = plot3([xend xend2(1,:)],[yend yend(2,:)],[zend zend(3,:)],'-','linewidth',3,'markersize',10,'color',[.6 .6 .3]);
        
        % Also plot the tip position of the robot, using hold on and hold
        % off, also keeping a handle to the plot so we can update the data
        % points later.
        hold on;
        htip = plot3(tip_history(1,i),tip_history(2,i),tip_history(3,i),'r.');
        hold off;
        
        % Label the axes.
        xlabel('X (in)');
        ylabel('Y (in)');
        zlabel('Z (in)');
        
        % Turn on the grid and the box.
        grid on;
        box on;
        
        % Set the axis limits.
        axis([-20 20 -20 20 -10 30])
        
        % Set the axis properties for 3D visualization, which makes one
        % unit the same in every direction, and enables rotation.
        axis vis3d;
        
        %view(0,90)
        
        % Put text on the plot to show how much time has elapsed.  The text
        % is centered.
        htime = text(1,1,1,sprintf('t = %.2f s',t(i)),'horizontalAlignment','center');
        
        % Add a title.
        title('Puma Robot by Valerie Cohen and Bahram Banisadr');
    else
        % Once the animation has been set up, we don't need to reformat the
        % whole plot.  We just set the data to the correct new values for
        % the robot animation, the tip history, and the text showing the
        % elapsed time.
        set(hrobot, 'color', [.3 .3 .3])
        if points_to_plot(3,end) < 0
            set(hrobot, 'color', [0 0 1]) %set to blue if touching table
        end
        if radtodeg(theta1) < -180 || radtodeg(theta1) > 110 ||...
                radtodeg(theta2) < -75 || radtodeg(theta2) > 240 ||...
                radtodeg(theta3) < -235 || radtodeg(theta3) > 60 ||...
                radtodeg(theta4) < -580 || radtodeg(theta4) > 40 ||...
                radtodeg(theta5) < -120 || radtodeg(theta5) > 110 ||...
                radtodeg(theta6) < -215 || radtodeg(theta6) > 295
            set(hrobot,'color',[0,1,0]) %set to green if beyond joint angle
            if points_to_plot(3,end) < 0
                set(hrobot, 'color', [1 0 1]) %set to (whatever color that makes) if beyond joint angle and touching table
            end
        end
        set(hrobot,'xdata',points_to_plot(1,:),'ydata',points_to_plot(2,:),'zdata',points_to_plot(3,:))
        set(htip,'xdata',tip_history(1,1:i),'ydata',tip_history(2,1:i),'zdata',tip_history(3,1:i))
        set(htime,'string', (sprintf('t = %.2f s',t(i))));
       % set(endeffector_x, 'xdata', [xend xend2(1,:)],'ydata',[yend yend(2,:)],'zdata',[zend zend(3,:)]);
    end
    
    
    % Pause for a short duration so that the viewer can watch animation evolve over time.
    pause(GraphingTimeDelay)

end

disp('Done with the animation.')

















