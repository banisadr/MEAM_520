function [tau1, tau2, tau3] = phantom_force_to_torques_weka_bahram(Fx, Fy, Fz, theta1, theta2, theta3)

% The inputs are the desired Cartesian force vector in N and the robot's
% present pose, specified as joint angles in radians.
% The outputs are joint torques in Nm.


%% Phantom Parameters
% lengths in mm
L1 = 170; 
L2 = 140;
L3 = 140;
b  = 35;
% lengths in m
L1 = L1*.001;
L2 = L2*.001;
L3 = L3*.001;
b = b*.001;

% Linear Velocity Jacobian
Jv = [-L2*sin(theta1)*cos(theta2) + L3*sin(theta1)*sin(theta3)          -L2*cos(theta1)*sin(theta2)               -L3*cos(theta1)*cos(theta3);
      L2*cos(theta1)*cos(theta2) - L3*cos(theta1)*sin(theta3)           -L2*sin(theta1)*sin(theta2)               -L3*sin(theta1)*cos(theta3);
      0                                                                 -L2*cos(theta2)                           L3*sin(theta3)];
% Force Vector
F = [Fx Fy Fz]';

% Torque Vector  
tau = Jv'*F;

tau1 = tau(1);
tau2 = tau(2);
tau3 = tau(3);
