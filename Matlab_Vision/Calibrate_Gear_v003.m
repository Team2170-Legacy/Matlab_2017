%
% Calibrate_Gear.m
% Written by Jacob Krucinski on 2/16/17

img_001 = imread('Image_001.png');
% imshow(img_001);    % RGB Image
imshow(uint8(255) - img_001);
hsv_img = rgb2hsv(img_001);

clc
disp('Click in the upper left and then the lower right of the Left gear target reflective tape...');

rect1 = ginput(2)
disp('Click in the upper left and then the lower right of the Left gear target reflective tape...');
rect2 = ginput(2)

rounded1 = round(rect1);
rounded2 = round(rect2);

% doesn't work img_001 has y-pixels first, x second target_L = img_001(rounded1(1,1):rounded1(2,1), rounded1(1,2):rounded1(2,2));
target_L = img_001( rounded1(1,2):rounded1(2,2) , rounded1(1,1):rounded1(2,1), : );
target_R = img_001( rounded2(1,2):rounded2(2,2) , rounded2(1,1):rounded2(2,1), : );

%% target_L pixel value averaging
hsv_target_L = rgb2hsv(target_L);

target_L_r = target_L(:,:,1);   % An array of all 'r' values in rgb target_l image
target_L_g = target_L(:,:,2);   % An array of all 'g' values in rgb target_l image
target_L_b = target_L(:,:,3);   % An array of all 'b' values in rgb target_l image

target_L_r_avg = mean(mean(target_L_r));   % Average of all target_L_r values
target_L_g_avg = mean(mean(target_L_g));   % Average of all target_L_g values
target_L_b_avg = mean(mean(target_L_b));   % Average of all target_L_b values

rounded_target_L_r_avg = round(target_L_r_avg);   % Rounded average of all target_L_r values
rounded_target_L_g_avg = round(target_L_g_avg);   % Rounded average of all target_L_g values
rounded_target_L_b_avg = round(target_L_b_avg);   % Rounded average of all target_L_b values

target_L_RGB_thresh = [rounded_target_L_r_avg rounded_target_L_g_avg rounded_target_L_b_avg]   % Rounded average rgb values in one single array


target_L_H = hsv_target_L(:,:,1) * 360;   % An array of all 'H' values in rgb target_l image, mult. by 359 due to 0-1 range for H
target_L_S = hsv_target_L(:,:,2) * 100;   % An array of all 'S' values in rgb target_l image, mult. by 100 for percent
target_L_V = hsv_target_L(:,:,3) * 100;   % An array of all 'V' values in rgb target_l image, mult. by 100 for percent

target_L_H_avg = mean(mean(target_L_H));   % Average of all target_L_r values
target_L_S_avg = mean(mean(target_L_S));   % Average of all target_L_g values
target_L_V_avg = mean(mean(target_L_V));   % Average of all target_L_b values

rounded_target_L_H_avg = round(target_L_H_avg);   % Rounded average of all target_L_r values
rounded_target_L_S_avg = round(target_L_S_avg);   % Rounded average of all target_L_g values
rounded_target_L_V_avg = round(target_L_V_avg);   % Rounded average of all target_L_b values

target_L_HSV_thresh = [rounded_target_L_H_avg rounded_target_L_S_avg rounded_target_L_V_avg]   % Rounded average rgb values in one single array

%% target_R pixel value averaging
hsv_target_R = rgb2hsv(target_R);

target_R_r = target_R(:,:,1);   % An array of all 'r' values in rgb target_R image
target_R_g = target_R(:,:,2);   % An array of all 'g' values in rgb target_R image
target_R_b = target_R(:,:,3);   % An array of all 'b' values in rgb target_R image

target_R_r_avg = mean(mean(target_R_r));   % Average of all target_R_r values
target_R_g_avg = mean(mean(target_R_g));   % Average of all target_R_g values
target_R_b_avg = mean(mean(target_R_b));   % Average of all target_R_b values

rounded_target_R_r_avg = round(target_R_r_avg);   % Rounded average of all target_R_r values
rounded_target_R_g_avg = round(target_R_g_avg);   % Rounded average of all target_R_g values
rounded_target_R_b_avg = round(target_R_b_avg);   % Rounded average of all target_R_b values

target_R_RGB_thresh = [rounded_target_R_r_avg rounded_target_R_g_avg rounded_target_R_b_avg]   % Rounded average rgb values in one single array


target_R_H = hsv_target_R(:,:,1) * 360;   % An array of all 'H' values in rgb target_R image, mult. by 359 due to 0-1 range for H
target_R_S = hsv_target_R(:,:,2) * 100;   % An array of all 'S' values in rgb target_R image, mult. by 100 for percent
target_R_V = hsv_target_R(:,:,3) * 100;   % An array of all 'V' values in rgb target_R image, mult. by 100 for percent

target_R_H_avg = mean(mean(target_R_H));   % Average of all target_R_r values
target_R_S_avg = mean(mean(target_R_S));   % Average of all target_R_g values
target_R_V_avg = mean(mean(target_R_V));   % Average of all target_R_b values

rounded_target_R_H_avg = round(target_R_H_avg);   % Rounded average of all target_R_r values
rounded_target_R_S_avg = round(target_R_S_avg);   % Rounded average of all target_R_g values
rounded_target_R_V_avg = round(target_R_V_avg);   % Rounded average of all target_R_b values

target_R_HSV_thresh = [rounded_target_R_H_avg rounded_target_R_S_avg rounded_target_R_V_avg]   % Rounded average rgb values in one single array

%% Calculate min and max of all target pixels
target_H    = [ target_R_H(:) ; target_L_H(:) ];
H_min       = min(target_H);
H_max       = max(target_H);
disp([ 'H_range = ' num2str(H_min) ' - ' num2str(H_max) ])

target_S    = [ target_R_S(:) ; target_L_S(:) ];
S_min       = min(target_S);
S_max       = max(target_S);
disp([ 'S_range = ' num2str(S_min) ' - ' num2str(S_max) ])

target_V    = [ target_R_V(:) ; target_L_V(:) ];
V_min       = min(target_V);
V_max       = max(target_V);
disp([ 'V_range = ' num2str(V_min) ' - ' num2str(V_max) ])

%% Plot figure windows
figure
imshow(target_L);
figure
imshow(target_R);