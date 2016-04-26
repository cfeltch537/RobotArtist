% Cody Feltch
% Test Script for scanning area and creating point cloud
% 4/24/2016
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear all;
close all;

%% Add mini vie direcctories to MATLAB path
miniVieDirs = genpath('C:\Users\Cody\Documents\GitHub\MiniVie')
addpath(miniVieDirs)

%% Start the Virtual Cyton Robot
close all;
doConnectToHW = true;
hwComPort = 'COM3';
hCyton = StartVirtualCyton(doConnectToHW, hwComPort);

%%

% create Cyton VIE
%hCyton = Presentation.CytonI.CytonI();
% allow instant updates by turning off real-time control timing
hCyton.hPlant.stop();

% set the arm to a bent position
% q = [0 0 0 -pi/4 0 -pi/4 0 0];
q = zeros(1,8);
hCyton.setJointParameters(q);
q = [0 pi/4 0 -pi/2 0 -pi/4 0 0];
hCyton.setJointParameters(q);

for i = 1:30  % iterate this command set 20 times
    dt = 0.05;
    pause(dt);

    % specify a motion in cartesian space: negative z-axis == down
    endpointVelocity = [100 0 0]';

    % get the current joint angle (since the jacobian depends on this)
    q = hCyton.JointParameters;

    % Compute jacobian
    J = numericJacobian(q);
    %J = hCyton.hControls.numericJacobian(q);
    J = J(1:3,:);  % consider position only

    % pseudo-inverse method for inverting non-square jacobian
    pinvJ = pinv(J);

    % joint angle velocities are the product of J^-1 * endpoint velocity
    q_dot = pinvJ * endpointVelocity;

    % set new joint values as position + incremental velocity
    qNew = q + [q_dot; 0]*dt;

    hCyton.setJointParameters(qNew);

end


