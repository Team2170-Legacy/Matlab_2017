Calib_Dir = pwd;
start_string = datestr(now,30);
Image_Dir = [ 'Calib_Images_' start_string ];
mkdir(Image_Dir);
cd(Image_Dir);

% cd Home_Images\;
% % cd(Calib_Dir);

% Get and set up video camera with 320x240 dimensions
vid = videoinput('winvideo', 2, 'YUY2_320x240');
src = getselectedsource(vid);
triggerconfig(vid, 'manual');
vid.FramesPerTrigger = 1;
vid.TriggerRepeat = 0;

src.ExposureMode = 'manual';
src.WhiteBalanceMode = 'manual';
%   Exposure range is -11 to 1, total number of exposure steps = 12, one
%   step = 8.3%.
%   We used exposure value 9 in the RoboRio C++ code to match exposure
%   between -10 (on PC) to 9 (RoboRio)
src.Exposure = -10;   %  Set exposure, REMINDER: CHECK TO SEE IF THIS MATCHES WITH C++ CODE AS WELL

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
        imwrite(getdata(vid), ['Calib_Image_00' num2str(i) '.png']);
        i = i + 1;
        w = waitforbuttonpress;
        % stoppreview(vid);
        % a = input('Enter 1 to continue, 0 to stop...: ')
end

% diskLogger = VideoWriter('C:\Users\User2\Documents\MATLAB\Calibration\Test1_0002.avi', 'Motion JPEG AVI');

% vid.DiskLogger = diskLogger;
imaqreset;
cd(Calib_Dir);