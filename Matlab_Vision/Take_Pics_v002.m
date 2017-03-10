% Get and set up video camera with 320x240 dimensions
vid = videoinput('winvideo', 2, 'YUY2_320x240');
src = getselectedsource(vid);
triggerconfig(vid, 'manual');
vid.FramesPerTrigger = 1;
vid.TriggerRepeat = 0;

src.WhiteBalanceMode = 'manual';
src.ExposureMode = 'manual';
src.Exposure = -9;   %  Set exposure, REMINDER: CHECK TO SEE IF THIS MATCHES WITH C++ CODE AS WELL

vid.ReturnedColorspace = 'rgb';

src.BacklightCompensation = 'on';

preview(vid);
%start(vid);

disp('Click mouse button to take an additional picture, or press a key to stop capture');

f = figure;
text(0,0, 'Press or click here...');
xlim([ -1 1 ])
ylim([ -1 1 ])
w = waitforbuttonpress;
i = 1;
% a = input('Enter 1 to continue, 0 to stop...: ')
while w == 0
        start(vid);
        trigger(vid);
        imwrite(getdata(vid), ['Seq_01_00' num2str(i) '.png']);
        i = i + 1;
        w = waitforbuttonpress;
        % stoppreview(vid);
        % a = input('Enter 1 to continue, 0 to stop...: ')
end

% diskLogger = VideoWriter('C:\Users\User2\Documents\MATLAB\Calibration\Test1_0002.avi', 'Motion JPEG AVI');

% vid.DiskLogger = diskLogger;
imaqreset;
