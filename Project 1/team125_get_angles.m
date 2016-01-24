function thetas = team125_get_angles(t)

% Define dance and all other variables needed multiple times to be persistent.
persistent tvia thetavia thetadotvia trajectorytypevia

% This function is initialized by calling it with no argument.
if (nargin == 0)
    
    % Load the team's dance file from disk.  It contains the variable dance.
    load team125
    
    % Pull the list of via point times out of the dance matrix.
    tvia = dance(:,1);
    
    % Pull the list of joint angles out of the dance matrix.
    thetavia = dance(:,2:7);
    
    % Pull the list of joint angle velocities out of the dance matrix.
    thetadotvia = dance(:,8:13);
    
    % Pull the list of trajectory types out of the dance matrix.
    trajectorytypevia = dance(:,14);
    
    % Return from initialization.
    return
    
end

% Determine which trajectory we should be executing.  Assuming the via
% point times are monotonically increasing, we look for the first via point
% time that is greater than the current time.  Subtract 1 to get to the
% index of the via point that starts this trajectory.
traj = find(t < tvia,1) - 1;

% Select the correct trajectory types.
switch (trajectorytypevia(traj))
    case 0
        % Cubicly interpolate between the via points 
        thetas = team125_cubic_trajectory(t,tvia(traj),tvia(traj+1),thetavia(traj,:)',thetavia(traj+1,:)',thetadotvia(traj,:)', thetadotvia(traj+1,:)');
    case 1
        % LBSP
        thetas = team125_lspb_trajectory(t,tvia(traj),tvia(traj+1),thetavia(traj,:)',thetavia(traj+1,:)',thetadotvia(traj,:)', thetadotvia(traj+1,:)');
    case 2
        % Linear
        thetas = team125_linear_trajectory(t,tvia(traj),tvia(traj+1),thetavia(traj,:)',thetavia(traj+1,:)');
    otherwise
        error(['Unknown trajectory type: ' num2str(trajectorytypevia(traj))])
end


