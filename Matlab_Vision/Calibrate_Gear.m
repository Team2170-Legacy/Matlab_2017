%
%   Calibrate_Gear.m
%   Jacob Krucinski (jacob1576@gmail.com)

img_001 = imread('Image_001.png');
imshow(img_001);

rect1 = ginput(2)
rect2 = ginput(2)

rounded1 = round(rect1);
rounded2 = round(rect2);

% doesn't work img_001 has y-pixels first, x second target_L = img_001(rounded1(1,1):rounded1(2,1), rounded1(1,2):rounded1(2,2));
target_L = img_001( rounded1(1,2):rounded1(2,2) , rounded1(1,1):rounded1(2,1), : );
target_R = img_001( rounded2(1,2):rounded2(2,2) , rounded2(1,1):rounded2(2,1), : );
figure
imshow(target_L);
figure
imshow(target_R);