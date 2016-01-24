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

variable1_deg = 5;
variable2_cm = 10;


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
title('Reachable Workspace by YOUR NAME and YOUR PARTNER''S NAME')


%% Plot the Reachable Workspace
% Update this code.

for i = 1:10
    for j = -5:5:15

        % Calculate x and y locations.  
        x = i;
        y = j + 0.5*i;
        
        % Plot this position on the graph, setting the color based on the location.
        if ((y > 12) && (x > 5))
            plot(x,y,'rx')
        else
            plot(x,y,'g+')
        end
            
    end
end
