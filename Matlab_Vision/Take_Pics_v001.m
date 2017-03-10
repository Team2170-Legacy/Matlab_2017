% Get and set up video camera with 320x240 dimensions
vid = videoinput('winvideo', 1, 'RGB24_320x240');
src = getselectedsource(vid);
triggerconfig(vid, 'manual');
vid.FramesPerTrigger = 1;
vid.TriggerRepeat = 0;

src.ExposureMode = 'manual';
src.Exposure = -2;   %  Set exposure, REMINDER: CHECK TO SEE IF THIS MATCHES WITH C++ CODE AS WELL

vid.ReturnedColorspace = 'rgb';

src.BacklightCompensation = 'on';

preview(vid);
%start(vid);

%w = waitforbuttonpress;
i = 1;
a = input('Enter 1 to continue, 0 to stop...: ')
while a
    %if w == 0 || w == 1
    start(vid);
    trigger(vid);
    imwrite(getdata(vid), ['C:\Users\User2\Documents\MATLAB\Calibration\Jacob_00' num2str(i) '.png']);
    i = i + 1;
    % stoppreview(vid);
    a = input('Enter 1 to continue, 0 to stop...: ')
    %end
end

% diskLogger = VideoWriter('C:\Users\User2\Documents\MATLAB\Calibration\Test1_0002.avi', 'Motion JPEG AVI');

% vid.DiskLogger = diskLogger;
imaqreset;
