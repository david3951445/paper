function MakeVideo(frame, fps, filename)
    disp('making .avi ...')
    %% save frame to .avi
    writerObj = VideoWriter(filename);
    writerObj.FrameRate = fps;
    open(writerObj);
    for i = 1 : length(frame)
        writeVideo(writerObj, frame{i});
    end
    close(writerObj);
end