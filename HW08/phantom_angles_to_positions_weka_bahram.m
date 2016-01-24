function [p1, p2, p3, p4] = phantom_angles_to_positions_weka_bahram(theta1, theta2, theta3)

% The inputs are joint angles in radians.
% The outputs are the xyz positions of points along the robot for plotting.  
% Each point should be a three-element column vector.  The last one you
% return should be the tip position. 

% Phantom Robot Parameters (lengths in mm).
L1 = 170; 
L2 = 140;
L3 = 140;
b  = 35;
% Origin of base in frame zero (homogeneous representation).
o = [0 0 0 1]';
p1 = [o(1) o(2) o(3)]';

%% Frame 1. 

% Origin of frame 1 w/ respect to frame 0.
d1_in_0 = [0 0 L1]';

% Rotation 90 degrees about x0-axis.
Rx0_90 = [1     0     0;
          0     0    -1;
          0     1     0];
      
% Rotation by -theta1 about z0-axis.
Rz0_negtheta1 = [cos(-theta1)   -sin(-theta1)     0;
                 sin(-theta1)    cos(-theta1)     0;
                 0                0               1];
          
% Rotation matrix, frame 1 w/ respect to frame 0.
R1_in_0 = Rz0_negtheta1 * Rx0_90;      

% Homogeneous transformation matrix, frame 1 in frame 0.
H1_in_0 = [R1_in_0(1,:) d1_in_0(1);
           R1_in_0(2,:) d1_in_0(2);
           R1_in_0(3,:) d1_in_0(3);
           0      0       0     1];
       
% Origin of Frame 1.
o1 = H1_in_0 * o;
p2 = [o1(1) o1(2) o1(3)]';

%% Frame 3.

% Origin of frame 3 w/ repect to frame 1.
 d3_in_1 = [L2*cos(theta2)    -L2*sin(theta2)     0     1]';

% Origin of frame 3 w/ respect to frame 0.
 d3_in_0 = H1_in_0 * d3_in_1;
 p3 = [d3_in_0(1) d3_in_0(2) d3_in_0(3)]';
 
%% End Effector.

% Origin of Frame 5 (tip) w/ respect to frame 1.
d5_in_1 = [(L2*cos(theta2) - L3*sin(theta3))     (-L2*sin(theta2) - L3*cos(theta3))     0     1]';

% Origin of Frame 5 w/ respect to frame 0.
d5_in_0 = H1_in_0 * d5_in_1;
p4 = [d5_in_0(1) d5_in_0(2) d5_in_0(3)]';





       