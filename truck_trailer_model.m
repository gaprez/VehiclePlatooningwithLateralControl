function [A, B, C] =  truck_trailer_model(Vx)
% To describe the lateral vehicle dynamics, this example uses a linearized
% model of a truck-trailer system

m1 = 9053;             % [kg] Mass of Tractor
I1 = 52161;            % [kg m^2] Yaw Moment of Inertia of Tractor
m2 = 27361;            % [kg] Trailer Mass
I2 = 767667;           % [kg m^2] Yaw Moment of Inertia of Trailer
l1 = 2.59;             % [m] Distance from tractor CG to steering axle
l2 = 2.70;             % [m] Distance from tractor CG to front axle
l3 = 4.02;             % [m] Distance from tractor CG to rear axle
l4 = 4.17;             % [m] Distance from trailer CG to front trailer axle
l5 = 5.41;             % [m] Distance from trailer CG to rear trailer axle
l6 = 3.36;             % [m] Distance from tractor CG to 5th wheel hitch
l7 = 6.32;             % [m] Distance from trailer CG to 5th wheel hitch

C1 = 150000;           % [N/rad] Cornering Stiffness Tractor 1
C2 = 121000;           % [N/rad] Cornering Stiffness Tractor 2
C3 = 121000;           % [N/rad] Cornering Stiffness Tractor 3
C4 = 121000;           % [N/rad] Cornering Stiffness Trailer 4
C5 = 121000;           % [N/rad] Cornering Stiffness Tractor 5

% Specify a state-space model, |G(s)|, of the lateral vehicle dynamics.
M = [m1 0 m2 0; 0 I1 -l6*m2 0; 0 0 -l7*m2 I2; 1 -l6 -1 -l6];
a = [(-C1-C2-C3)/Vx ((-C1*l1+C2*l2+C3*l3)/Vx)-m1*Vx (-C4-C5)/Vx...
    ((C4*l4+C5*l5)/Vx)-m2*Vx;(-C1*l1+C2*l2+C3*l3)/Vx ...
    (-C1*l1^2 - C2*l2^2 - C3*l3^2)/Vx ...
    (-C4*l6-C5*l6)./Vx l6*((-C4*l4-C5*l5)/Vx + m2*Vx);...
    0 0 (C4*l7+C5*l7+C4*l4+C5*l5)/Vx ...
    ((-C4*l4*l7-C5*l5*l7-C4*l4^2-C5*l5^2)/Vx+l7*m2*Vx);...
    0 -Vx 0 Vx];
b = [C1; C1*l1; 0; 0];
C = [1 0 0 0; 0 1 0 0];

A = M\a;
B = M\b;
end