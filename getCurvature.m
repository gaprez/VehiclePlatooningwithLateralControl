function md = getCurvature(XRef,YRef,time)
% Get previewed curvature from desired X and Y positions for LKA
%
% Inputs:
%   IC: Initial Condition Object w/ Parameters: ax, Vx, profile
%       ax      -- Longitudinal Acceleration [m/s^2]
%       Vx      -- Longitudinal Velocity     [m/s]
%       profile -- Sinusoidal or Constant Acceleration Profile
%   time: time vector
%
% Outputs:
%   md: previewed curvature

% % Desired curvature
k_est = LineCurvature2D([XRef YRef]);
k_est(isnan(k_est)) = 0;

md.time = time;
md.signals.values = k_est;