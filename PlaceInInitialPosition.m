function PlaceInInitialPosition(hCyton)
    % Places the Robot in the initial bent position, faceing in the x-dir 
    hCyton.hPlant.stop();
    q = [0 pi/4 0 -pi/2 0 -pi/4 0 0];
    hCyton.setJointParameters(q);
end