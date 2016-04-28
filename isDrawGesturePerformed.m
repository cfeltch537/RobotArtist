function [ draw ] = isDrawGesturePerformed()
%ISDRAWGESTUREPERFORMED This should make use of the My EMG Data and return
%true if the drawing gesture is being performed.
    global SignalSource;
    % get the data
    chIds = [1 8];  
    % select channels for hand open and close signals
    SignalSource.getData();
    emgData = SignalSource.getFilteredData(100,chIds);  
    % returns
    % handle 2 channels
    rmsData = rms(emgData);    
    %<--adjust channel threshold for hand open
    threshClose = 0.2;      
    % <--adjust channel threshold for hand close
    if rmsData(2) > threshClose
        startDraw = 1;
        disp('Start Gesture')
    else
        startDraw = 0; 
        disp('End Gesture')
    end
    draw = startDraw;
end

