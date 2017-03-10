%
% Calibrate_Gear.m
% Written by Jacob Krucinski on 2/16/17

H_margin = 5;
S_margin = 5;
L_margin = 20;

img_001 = imread('Image_001.png');
% imshow(img_001);    % RGB Image
imshow(uint8(255) - img_001);
hsv_img = rgb2hsv(img_001);

clc
disp('Click in the upper left and then the lower right of the Left gear target reflective tape...');
rect1 = ginput(2)
disp('Click in the upper left and then the lower right of the Left gear target reflective tape...');
rect2 = ginput(2)
% rect1 = [108.0000 161.0000; 
%   122.0000  216.0000];
% 
% rect2 = [204.0000  164.0000;
%   216.0000  211.0000];
% 

rounded1 = round(rect1);
rounded2 = round(rect2);

% doesn't work img_001 has y-pixels first, x second target_L = img_001(rounded1(1,1):rounded1(2,1), rounded1(1,2):rounded1(2,2));
target_L = img_001( rounded1(1,2):rounded1(2,2) , rounded1(1,1):rounded1(2,1), : );
target_R = img_001( rounded2(1,2):rounded2(2,2) , rounded2(1,1):rounded2(2,1), : );

%% target_L pixel value averaging
hsv_target_L = rgb2hsv(target_L);

target_L_H = (hsv_target_L(:,:,1) * 360) / 2;   % An array of all 'H' values in rgb target_L image, mult. by 359 due to 0-1 range for H
target_L_S = hsv_target_L(:,:,2) * 255;   % An array of all 'S' values in rgb target_L image, mult. by 100 for percent
target_L_L = hsv_target_L(:,:,3) * 100;   % An array of all 'V' values in rgb target_R image, mult. by 100 for percent

target_L_H_avg = mean(mean(target_L_H));   % Average of all target_R_r values
target_L_S_avg = mean(mean(target_L_S));   % Average of all target_R_g values
target_L_L_avg = mean(mean(target_L_L));   % Average of all target_R_b values

target_L_HSL_thresh = [target_L_H_avg target_L_S_avg target_L_L_avg]   % Rounded average rgb values in one single array

%% target_R pixel value averaging
hsv_target_R = rgb2hsv(target_R);

target_R_H = (hsv_target_R(:,:,1) * 360) / 2;   % An array of all 'H' values in rgb target_R image, mult. by 359 due to 0-1 range for H
target_R_S = hsv_target_R(:,:,2) * 255;   % An array of all 'S' values in rgb target_R image, mult. by 100 for percent
target_R_L = hsv_target_R(:,:,3) * 100;   % An array of all 'V' values in rgb target_R image, mult. by 100 for percent

target_R_H_avg = mean(mean(target_R_H));   % Average of all target_R_r values
target_R_S_avg = mean(mean(target_R_S));   % Average of all target_R_g values
target_R_L_avg = mean(mean(target_R_L));   % Average of all target_R_b values

target_R_HSL_thresh = [target_R_H_avg target_R_S_avg target_R_L_avg]   % Rounded average rgb values in one single array

%% Calculate min and max of all target pixels
target_H    = [ target_R_H(:) ; target_L_H(:) ];
H_min       = min(target_H);
H_max       = max(target_H);
disp([ 'H_range = ' num2str(H_min) ' - ' num2str(H_max) ])

target_S    = [ target_R_S(:) ; target_L_S(:) ];
S_min       = min(target_S);
S_max       = max(target_S);
disp([ 'S_range = ' num2str(S_min) ' - ' num2str(S_max) ])

target_L_val    = [ target_R_L(:) ; target_L_L(:) ];
L_min       = min(target_L_val);
L_max       = max(target_L_val);
disp([ 'L_range = ' num2str(L_min) ' - ' num2str(L_max) ])

H_treshold  = max( 0, min( 180, [ (H_min - H_margin) (H_max + H_margin) ]))
S_treshold  = max( 0, min( 255, [ (S_min - S_margin) (S_max + S_margin) ]))
L_treshold  = max( 0, min( 255, [ (L_min - L_margin) (L_max + L_margin) ]))

%% Plot figure windows
figure
imshow(target_L);
figure
imshow(target_R);