% Cody Feltch
% Test Script for scanning area and creating point cloud
% 4/24/2016
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear all;
close all;

%% Add mini vie direcctories to MATLAB path
miniVieDirs = genpath('C:\Users\Cody\Documents\GitHub\MiniVie')
addpath(miniVieDirs)

global inTestMode SignalSource;

SignalSource = Inputs.MyoUdp.getInstance();
SignalSource.initialize();
SignalSource.NumSamples = 100;

%% Start the Virtual Cyton Robot

% Initialize Arduino unoArduinoand Cyton
hCyton = StartVirtualCyton(false,'COM3');
unoArduino  = arduino('COM5', 'uno')
inTestMode = false;

%% 

% Place in initial search Position, Then move to canvas on button press
PlaceInInitialPosition(hCyton)
    waitforbuttonpress
MoveToHoldDistance(hCyton, unoArduino)
    waitforbuttonpress

while true
    if inTestMode == false
        draw = isDrawGesturePerformed();
        while(draw)
            MoveToDrawDistance(hCyton, unoArduino)
            DrawLoop(hCyton)
        end
    else
        MoveToDrawDistance(hCyton, unoArduino)
        DrawLoop(hCyton)
    end
end

%%




% %%
% for i = 1:30  % iterate this command set 20 times
%     dt = 0.05;
%     pause(dt);
% 
%     % specify a motion in cartesian space: negative z-axis == down
%     endpointVelocity = [100 0 0]';
% 
%     % get the current joint angle (since the jacobian depends on this)
%     q = hCyton.JointParameters;
% 
%     % Compute jacobian
%     J = numericJacobian(q);
%     %J = hCyton.hControls.numericJacobian(q);
%     J = J(1:3,:);  % consider position only
% 
%     % pseudo-inverse method for inverting non-square jacobian
%     pinvJ = pinv(J);
% 
%     % joint angle velocities are the product of J^-1 * endpoint velocity
%     q_dot = pinvJ * endpointVelocity;
% 
%     % set new joint values as position + incremental velocity
%     qNew = q + [q_dot; 0]*dt;
% 
%     hCyton.setJointParameters(qNew); 
% end


