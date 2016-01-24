function thetas = team205_puma_ik(x, y, z, phi, theta, psi)
%% team205_puma_ik.m
%
% Calculates the full inverse kinematics for the PUMA 260.
%
% This Matlab file provides the starter code for the PUMA 260 inverse
% kinematics function of project 2 in MEAM 520 at the University of
% Pennsylvania.  The original was written by Professor Katherine J.
% Kuchenbecker. Students will work in teams modify this code to create
% their own script. Post questions on the class's Piazza forum. 
%
% The first three input arguments (x, y, z) are the desired coordinates of
% the PUMA's end-effector tip in inches, specified in the base frame.  The
% origin of the base frame is where the first joint axis (waist) intersects
% the table. The z0 axis points up, and the x0 axis points out away from
% the robot, perpendicular to the front edge of the table.  These arguments
% are mandatory.
%
%     x: x-coordinate of the origin of frame 6 in frame 0, in inches
%     y: y-coordinate of the origin of frame 6 in frame 0, in inches
%     z: z-coordinate of the origin of frame 6 in frame 0, in inches
%
% The fourth through sixth input arguments (phi, theta, psi) represent the
% desired orientation of the PUMA's end-effector in the base frame using
% ZYZ Euler angles in radians.  These arguments are mandatory.
%
%     phi: first ZYZ Euler angle to represent orientation of frame 6 in frame 0, in radians
%     theta: second ZYZ Euler angle to represent orientation of frame 6 in frame 0, in radians
%     psi: third ZYZ Euler angle to represent orientation of frame 6 in frame 0, in radians
%
% The output (thetas) is a matrix that contains the joint angles needed to
% place the PUMA's end-effector at the desired position and in the desired
% orientation. The first row is theta1, the second row is theta2, etc., so
% it has six rows.  The number of columns is the number of inverse
% kinematics solutions that were found; each column should contain a set
% of joint angles that place the robot's end-effector in the desired pose.
% These joint angles are specified in radians according to the
% order, zeroing, and sign conventions described in the documentation.  If
% this function cannot find a solution to the inverse kinematics problem,
% it will pass back NaN (not a number) for all of the thetas.
%
% Please change the name of this file and the function declaration on the
% first line above to include your team number rather than 200.


%% CHECK INPUTS

% Look at the number of arguments the user has passed in to make sure this
% function is being called correctly.
if (nargin < 6)
    error('Not enough input arguments.  You need six.')
elseif (nargin == 6)
    % This the correct way to call this function, so we don't need to do
    % anything special.
elseif (nargin > 6)
    error('Too many input arguments.  You need six.')
end


%% CALCULATE INVERSE KINEMATICS SOLUTION(S)

% For now, just set the first solution to NaN (not a number) and the second
% to zero radians.  You will need to update this code.
% NaN is what you should output if there is no solution to the inverse
% kinematics problem for the position and orientation that were passed in.
% For example, this would be the correct output if the desired position for
% the end-effector was outside the robot's reachable workspace.  We use
% this sentinel value of NaN to be sure that the code calling this function
% can tell that something is wrong and shut down the PUMA.


cphi=cos(phi);
sphi=sin(phi);
ctheta=cos(theta);
stheta=sin(theta);
cpsi=cos(psi);
spsi=sin(psi);

r=[cphi*ctheta*cpsi-sphi*spsi -cphi*ctheta*spsi-sphi*cpsi cphi*stheta;
    sphi*ctheta*cpsi+cphi*spsi -sphi*ctheta*spsi+cphi*cpsi sphi*stheta;
    -stheta*cpsi stheta*spsi ctheta];

% Check determinant to make sure it's one.
det(r);

%Define link lengths
a=13;
b=2.5;
c=8;
d=2.5;
e=8.0;
f=2.5;

%find the wrist center
oc = [x y z]' - f*r(:,3);

ox = oc(1);
oy = oc(2);
oz = oc(3);


l = sqrt(ox^2+oy^2);
Lx = real(sqrt(l^2-(b+d)^2));
Ly = b+d;
alpha_1 = atan2(Ly,Lx);
gamma = pi/2-alpha_1;

theta1_1 = atan2(oy,ox)-alpha_1;
theta1_2 = theta1_1-2*gamma;


%basically, we need to make the bottom left corner of the r36 matrix whcih
%is just cos(theta5) equal to the bottom right corner of r03'*r06 which is
%just our r matrix. our r03 we should find using her kinematics


R = Lx; 
h = oz-a;
lambda = sqrt(R^2+h^2);
s3 = (lambda^2 -c^2-e^2)/(-2*c*e);

theta3_1 = atan2(s3,sqrt(1-s3^2));
theta3_2 = atan2(s3,-sqrt(1-s3^2));

theta2_l_1 = atan2(h,R)-atan2(e*cos(theta3_1),c-e*sin(theta3_1));
theta2_l_2 = atan2(h,R)-atan2(e*cos(theta3_2),c-e*sin(theta3_2));
theta2_r_1 = pi-(atan2(h,R)-atan2(e*cos(theta3_2),c-e*sin(theta3_2)));
theta2_r_2 = pi-(atan2(h,R)-atan2(e*cos(theta3_1),c-e*sin(theta3_1)));


th1 = [theta1_1     theta1_1    theta1_1    theta1_1    theta1_2    theta1_2    theta1_2    theta1_2];
th2 = [theta2_l_1   theta2_l_1  theta2_l_2  theta2_l_2  theta2_r_1  theta2_r_1  theta2_r_2  theta2_r_2];
th3 = [theta3_1     theta3_1    theta3_2    theta3_2    theta3_1    theta3_1    theta3_2    theta3_2];

th4 = zeros(1,8);
th5 = zeros(1,8);
th6 = zeros(1,8);

for i = 1:4
    

    A1 = dh_kuchenbe(0,(pi/2),a,th1(2*i));
    A2 = dh_kuchenbe(c,0,-b,th2(2*i));
    A3 = dh_kuchenbe(0,(-pi/2),-d,th3(2*i));
    R03=A1*A2*A3;
    R03=R03(1:3,1:3);
    
    R36_numerical = R03'*r;
    
    theta_star1 = atan2(sqrt(1-R36_numerical(3,3)^2),R36_numerical(3,3));
    theta_star2 = atan2(-sqrt(1-R36_numerical(3,3)^2),R36_numerical(3,3));
    phi_star1 = atan2(R36_numerical(2,3),R36_numerical(1,3));
    phi_star2 = atan2(-R36_numerical(2,3),-R36_numerical(1,3));
    psi_star1 = atan2(R36_numerical(3,2),-R36_numerical(3,1));
    psi_star2 = atan2(-R36_numerical(3,2),R36_numerical(3,1));
    
    th5(2*i-1) = -theta_star1;
    th5(2*i) = -theta_star2;
    
    th4(2*i-1) = phi_star1;
    th4(2*i) = phi_star2;
    
    th6(2*i-1) = psi_star1;
    th6(2*i) = psi_star2;

end





%% FORMAT OUTPUT

% Put all of the thetas into a column vector to return.
thetas = [th1; th2; th3; th4; th5; th6];

% By the very end, each column of thetas should hold a set of joint angles
% in radians that will put the PUMA's end-effector in the desired
% configuration.  If the desired configuration is not reachable, set all of
% the joint angles to NaN.