function [ output_args ] = MoveInGlobalX( hCyton, dt )
%MOVEINGLOBALX Summary of this function goes here
%   Detailed explanation goes here

        % dt = 0.05
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
        
        pause(dt);

end

