function [hCyton] = StartVirtualCyton( connectToHardware , comPortString)
%STARTVIRTUALCYTON Summary of this function goes here
%   Start the virtual cytin robot, and connect to the hardcoded com port if
%   input is true.

    import Presentation.CytonI.*
    delete(instrfindall)
    % Launch the cyton virtual integration environment (VIE)
    hCyton = CytonI
    % Connect to Hardware
    if connectToHardware == 1
        hCyton.connectToHardware(comPortString)
    end

end

