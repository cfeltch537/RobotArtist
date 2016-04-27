function DrawLoop( hCyton , uno )
% This should read the myo gyroscopic and accelerometry data, then move the
% end effector as dictated by the data

    global inTestMode;
    
    if inTestMode == true
        dt = 0.25;
        MoveInGlobal( hCyton, dt, [0,100,0]' )
        MoveInGlobal( hCyton, dt, [0,0,100]' )
        MoveInGlobal( hCyton, dt, [0,-100,0]' )
        MoveInGlobal( hCyton, dt, [0,0,-100]' )
    else
        
    end

end

