function Xdot = compute_motor_derivatives_weka_bahram(t,X)


% Define parameters of the Maxon 118743 motor, derived from data sheet.
kt = 23.467E-3; % N m/A
J = 10.3/1000/(100)^2; % kg*m^2
i_no_load = .026; % amps
omega_no_load = 508.938; % rad/s
L = .00024; % H
R = 2.18; % ohms

% We found the value of Bm by taking the no load case (when taul = 0).
% We were then able to set the sum of the torque on motor equal to zero
% since the motor is not accelerating (omegadot =0) in the no load case.
% Using no-load values from the motor data sheet, we were able to solve
% for Bm.
Bm = kt*i_no_load/omega_no_load; % N*m/(rad/s)

% Pull states out of state vector.
thetam = X(1); % Motor angle in radians.
omegam = X(2); % Motor angular velocity in radians per second.
ia = X(3); % Armature current in amps.


% Declare global variables so we can access their values.
global Vperiod Vpulse

% Calculate the voltage input at this instant in time.
% V is the voltage you should apply to the motor.
if (mod(t,Vperiod) > (Vperiod/2))
    V = Vpulse;
else
    V = 0;
end

% Set the test type.
testtype = 1; % Type of test to run.  0 is no load.  1 is stall.

% Check the test type.
if testtype
    % Stall test.  Set the load torque to be a torsional spring and damper,
    % as specified in the assignment.
    taul = 2*thetam + .0005*omegam;
else
    % No-load test.  Set the load torque to be zero.
    taul = 0;
end


% Calculate the derivative of each of our states, based on the input
% voltage, the motor's present state, and the motors electro-mechanical
% dynamics.

thetamdot = omegam;
omegamdot = (kt*ia -taul - Bm*omegam)/J;
iadot = (V - R*ia - kt*omegam)/L; 


% Create state derivative vector.
Xdot = [thetamdot ; omegamdot ; iadot];