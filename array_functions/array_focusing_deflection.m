function [mopaus] = array_focusing_deflection(mopaus)
% Array control system: Beam focusing and deflection
% 11/Nov/2025, Yanwei Huang (mrhuangyw1998@gmail.com) 
% Parameters
%   mopaus: 

% time delays for beam focusing 
if mopaus.source_focus ~= 0
    mopaus.time_delays = -(sqrt(mopaus.bundle_coord(:,1).^2 + mopaus.bundle_coord(:,2).^2 + mopaus.source_focus.^2) - mopaus.source_focus) ./ mopaus.speed;
    mopaus.time_delays = mopaus.time_delays - min(mopaus.time_delays); % Note: time delay values must be positive
else
    mopaus.time_delays = zeros(mopaus.element_num,1);
end

% Time delay for helical wavefront
[mopaus] = array_helical_wf(mopaus);

% time delays for beam deflection 
element_pitch_azi = mopaus.element_pitch; 
element_pitch_ele = mopaus.element_pitch*cosd(30); 
phase_step_azi = 2*pi*element_pitch_azi*sind(mopaus.source_deflect_azi)/(mopaus.speed/mopaus.source_f0); % phase step angle, azimuthal (axis-x)
phase_step_ele = 2*pi*element_pitch_ele*sind(mopaus.source_deflect_ele)/(mopaus.speed/mopaus.source_f0); % phase step angle, elevation (axis-y)

% Note: always use the top left element (i=1) as the reference when calculating steering phase delay
for i = 1:mopaus.element_num 
    % Azimuthal time delay calculation
    mopaus.time_delays(i) = mopaus.time_delays(i)+((mopaus.bundle_coord(i,1)-mopaus.bundle_coord(1,1))/element_pitch_azi)*phase_step_azi/(2*pi)/mopaus.source_f0; 
    
    % Elevation time delay calculation
    mopaus.time_delays(i) = mopaus.time_delays(i)+((mopaus.bundle_coord(i,2)-mopaus.bundle_coord(1,2))/element_pitch_ele)*phase_step_ele/(2*pi)/mopaus.source_f0;
    
end 

end

