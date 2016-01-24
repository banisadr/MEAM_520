function thetas = team205_choose_solution(allSolutions, thetasnow)
%% team205_choose_solution.m
%
% Chooses the best inverse kinematics solution from all of the solutions
% passed in.  This decision is based both on the characteristics of the
% PUMA 260 robot and on the robot's current configuration.
%
% This Matlab file provides the starter code for the solution choice
% function of project 2 in MEAM 520 at the University of Pennsylvania.  The
% original was written by Professor Katherine J. Kuchenbecker. Students
% will work in teams modify this code to create their own script. Post
% questions on the class's Piazza forum.
%
% The first input (allSolutions) is a matrix that contains the joint angles
% needed to place the PUMA's end-effector at the desired position and in
% the desired orientation. The first row is theta1, the second row is
% theta2, etc., so it has six rows.  The number of columns is the number of
% inverse kinematics solutions that were found; each column should contain
% a set of joint angles that place the robot's end-effector in the desired
% pose. These joint angles are specified in radians according to the 
% order, zeroing, and sign conventions described in the documentation.  If
% the IK function could not find a solution to the inverse kinematics problem,
% it will pass back NaN (not a number) for all of the thetas.
%
%    allSolutions: IK solutions for all six joints, in radians
%
% The second input is a vector of the PUMA robot's current joint angles
% (thetasnow) in radians.  This information enables this function to
% choose the solution that is closest to the robot's current pose. 
%
%     thetasnow: current values of theta1 through theta6, in radians
%
% Please change the name of this file and the function declaration on the
% first line above to include your team number rather than 200.


% You will need to update this function so it chooses intelligently from
% the provided solutions to choose the best one.
%
% There are several reasons why one solution might be better than the
% others, including how close it is to the robot's current configuration
% and whether it violates or obeys the robot's joint limits.
%
% Note that some of the PUMA's joints wrap around, while your solutions
% probably include angles only from -pi to pi or 0 to 2*pi.  If a joint
% wraps around, there can be multiple ways for the robot to achieve the
% same IK solution (the given angle as well as the given angle plus or
% minus 2*pi*n). Be careful about this point.

% For now, just return the last column of allSolutions.

dist = zeros(1,8);

for i = 1:8
    if (allSolutions(1,i) > deg2rad(-180+360) && allSolutions(1,i) < deg2rad(110+360))
        allSolutions(1,i) = allSolutions(1,i) - 2*pi;
    end
    if (allSolutions(1,i) > deg2rad(-180-360) && allSolutions(1,i) < deg2rad(110-360))
        allSolutions(1,i) = allSolutions(1,i) + 2*pi;
    end
    if allSolutions(1,i) < deg2rad(-180) || allSolutions(1,i) > deg2rad(110)
        allSolutions(:,i) = NaN;
    end
    
    if (allSolutions(2,i) > deg2rad(-75+360) && allSolutions(2,i) < deg2rad(240+360))
        allSolutions(2,i) = allSolutions(2,i) - 2*pi;
    end
    if (allSolutions(2,i) > deg2rad(-75-360) && allSolutions(2,i) < deg2rad(240-360))
        allSolutions(2,i) = allSolutions(2,i) + 2*pi;
    end
    if allSolutions(2,i) < deg2rad(-75) || allSolutions(2,i) > deg2rad(240)
        allSolutions(:,i) = NaN;
    end
    
    if (allSolutions(3,i) > deg2rad(-235+360) && allSolutions(3,i) < deg2rad(60+360))
        allSolutions(3,i) = allSolutions(3,i) - 2*pi;
    end
    if (allSolutions(3,i) > deg2rad(-235-360) && allSolutions(3,i) < deg2rad(60-360))
        allSolutions(3,i) = allSolutions(3,i) + 2*pi;
    end
    if allSolutions(3,i) < deg2rad(-235) || allSolutions(3,i) > deg2rad(60)
        allSolutions(:,i) = NaN;
    end
    
    if (allSolutions(4,i) > deg2rad(-580+360) && allSolutions(4,i) < deg2rad(40+360))
        allSolutions(4,i) = allSolutions(4,i) - 2*pi;
    end
    if (allSolutions(4,i) > deg2rad(-580-360) && allSolutions(4,i) < deg2rad(40-360))
        allSolutions(4,i) = allSolutions(4,i) + 2*pi;
    end
    if allSolutions(4,i) < deg2rad(-580) || allSolutions(4,i) > deg2rad(40)
        allSolutions(:,i) = NaN;
    end
    
    if (allSolutions(5,i) > deg2rad(-120+360) && allSolutions(5,i) < deg2rad(110+360))
        allSolutions(5,i) = allSolutions(5,i) - 2*pi;
    end
    if (allSolutions(5,i) > deg2rad(-120-360) && allSolutions(5,i) < deg2rad(110-360))
        allSolutions(5,i) = allSolutions(5,i) + 2*pi;
    end
    if allSolutions(5,i) < deg2rad(-120) || allSolutions(5,i) > deg2rad(110)
        allSolutions(:,i) = NaN;
    end
    
    if (allSolutions(6,i) > deg2rad(-215+360) && allSolutions(6,i) < deg2rad(295+360))
        allSolutions(6,i) = allSolutions(6,i) - 2*pi;
    end
    if (allSolutions(6,i) > deg2rad(-215-360) && allSolutions(6,i) < deg2rad(295-360))
        allSolutions(6,i) = allSolutions(6,i) + 2*pi;
    end
    if allSolutions(6,i) < deg2rad(-215) || allSolutions(6,i) > deg2rad(295)
        allSolutions(:,i) = NaN;
    end
    
    if ~isnan(allSolutions(1,i))
        dist(i) = dist(i) + sum(abs(allSolutions(:,i)-thetasnow));
    else
        dist(i) = NaN;
    end
end

index = find(dist == min(dist));


thetas = allSolutions(:,index);