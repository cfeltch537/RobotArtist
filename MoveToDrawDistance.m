function MoveToDrawDistance(hCyton, uno)
    % Places the Robot in the initial bent position, faceing in the x-dir 
    global inTestMode;
    
    % In Test Mode Move, move to the arbitrarily set end effector x
    % location of 200
    if inTestMode == true
        endEffectorXYZ = CalculateEndEffectorLocation(hCyton.JointParameters')
        endEffectorX = endEffectorXYZ(1)
        while endEffectorX < 225
            MoveInGlobalX(hCyton, 0.05)
            endEffectorXYZ = CalculateEndEffectorLocation(hCyton.JointParameters')
            endEffectorX = endEffectorXYZ(1)
        end
    % When not in test mode move foreward until the IR sensor output shows
    % that the canvas is close to you
    else
        voltage = uno.readVoltage(0)
        while voltage > 1.6
            MoveInGlobalX(hCyton, 0.05)
            voltage = uno.readVoltage(0)
        end
    end
   
end