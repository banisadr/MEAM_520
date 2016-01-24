%% MEAM 520 Homework 1
%
% Starter code written by Professor Katherine J. Kuchenbecker
% University of Pennsylvania
%
% The goal of this assignment is to plot the reachable workspace of the
% human arm in the horizontal plane, treating it like an RR manipulator.

% Clear all of the existing variables.
clear


%% Define Variables
% Add your own variables here.

elbowmin_deg = 15;
elbowmax_deg = 180;
shouldermin_deg = 15;
shouldermax_deg = 190;
forarmlength_cm = 29;
humerouslength_cm = 35;


%% Set Up Plot

figure(1)
clf

% Plot the shoulder as a black circle at the origin.
plot(0,0,'ko')

% Turn hold on so that we can plot additional things on this figure.
hold on

% Force the axes to be displayed as the same scale.
axis equal

% Label the axes, with units.
xlabel('X Position of Hand (cm)')
ylabel('Y Position of Hand (cm)')

% Set the title of the plot.
title('Reachable Workspace by Bahram')


%% Plot the Reachable Workspace
% Update this code.

for i = shouldermin_deg:5:shouldermax_deg
    for j = elbowmin_deg:5:shouldermax_deg

        % Calculate x and y locations.  
        x = humerouslength_cm*cosd(i)+ forarmlength_cm*cosd(i+j);
        y = humerouslength_cm*sind(i)+ forarmlength_cm*sind(i+j);
        
        % Plot this position on the graph, setting the color based on the location.
        if ((y < 5) && (x < 0))
            plot(x,y,'rx')
        else
            plot(x,y,'g+')
        end
            
    end
end
