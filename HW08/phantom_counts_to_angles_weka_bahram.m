function [theta1, theta2, theta3] = phantom_counts_to_angles_weka_bahram(Q1, Q2, Q3)

% The inputs are encoder values in counts.
% The outputs are joint angles in radians.

% Encoder lines/counts per revolution. (CPR)
n = 1000; %radians/count

% Calculate encoder resolution.
delta = 2*pi/(4*n);

% Drum diameters (mm)
d_d1 = 115;
d_d2 = 70;
d_d3 = d_d2;

% Motor Shaft Diameters (mm)
d_c1 = 15;
d_c2 = 10;
d_c3 = d_c2;

% Gear Ratios
rho1 = d_d1/d_c1;
rho2 = d_d2/d_c2;
rho3 = d_d3/d_c3;

% Joint angles
theta1 = delta*Q1/rho1;
theta2 = delta*Q2/rho2;
theta3 = delta*Q3/rho3;