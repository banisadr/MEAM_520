function [V1, V2, V3] = phantom_torques_to_voltages_weka_bahram(tau1, tau2, tau3)

% The inputs are joint torques in Nm.
% The outputs are DAC voltages in V.

% Torque constant (Nm)
kt = .0234;

% Maximum current (Amps)
i_max = 1.250 ;

% Resistance (ohms)
R = 5.00;

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

% Motor Torques
tau_m1 = tau1/rho1;
tau_m2 = tau2/rho2;
tau_m3 = tau3/rho3;

% Joint Currents

i1 = tau_m1/kt;
i2 = tau_m2/kt;
i3 = tau_m3/kt;

if i1 > i_max
    i1 = i_max;
end

if i2 > i_max
    i2 = i_max;
end

if i2 > i_max
    i2 = i_max;
end

% Joint Voltages
V1 = i1*R;
V2 = i2*R;
V3 = i3*R;





