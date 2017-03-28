%
% Calibrate_Gear_v008.m
% Written by Jacob Krucinski on 2/16/17
% Updated 03/27/2017 by Martin Krucinski to switch from HSL to HSV and
% updated H/S/V margins and MinRation & MaxRatio

H_margin = 5;
S_margin = 10;
V_margin = 20;

final_H_threshold = [0 0];
final_S_threshold = [0 0];
final_V_threshold = [0 0];
final_h          = 1000;
final_w          = 1000;

for img=1:3
    if img == 1
        img_001 = imread('Calib_Image_001.png');
        imshow(uint8(255) - img_001);
        hsv_img = rgb2hsv(img_001);
        
        disp('Click in the upper left and then the lower right of the Left gear target reflective tape...');
        rect1 = ginput(2)
        disp('Click in the upper left and then the lower right of the Left gear target reflective tape...');
        rect2 = ginput(2)
        
        rounded1 = round(rect1);
        rounded2 = round(rect2);
        
        target_L = img_001( rounded1(1,2):rounded1(2,2) , rounded1(1,1):rounded1(2,1), : );
        target_R = img_001( rounded2(1,2):rounded2(2,2) , rounded2(1,1):rounded2(2,1), : );
    end
    
    if img == 2
        img_002 = imread('Calib_Image_002.png');
        imshow(uint8(255) - img_002);
        hsv_img = rgb2hsv(img_002);
        
        disp('Click in the upper left and then the lower right of the Left gear target reflective tape...');
        rect1 = ginput(2)
        disp('Click in the upper left and then the lower right of the Left gear target reflective tape...');
        rect2 = ginput(2)
        
        rounded1 = round(rect1);
        rounded2 = round(rect2);
        
        target_L = img_002( rounded1(1,2):rounded1(2,2) , rounded1(1,1):rounded1(2,1), : );
        target_R = img_002( rounded2(1,2):rounded2(2,2) , rounded2(1,1):rounded2(2,1), : );
    end
    
    if img == 3
        img_003 = imread('Calib_Image_003.png');
        imshow(uint8(255) - img_003);
        hsv_img = rgb2hsv(img_003);
        
        disp('Click in the upper left and then the lower right of the Left gear target reflective tape...');
        rect1 = ginput(2)
        disp('Click in the upper left and then the lower right of the Left gear target reflective tape...');
        rect2 = ginput(2)
        
        rounded1 = round(rect1);
        rounded2 = round(rect2);
        
        target_L = img_003( rounded1(1,2):rounded1(2,2) , rounded1(1,1):rounded1(2,1), : );
        target_R = img_003( rounded2(1,2):rounded2(2,2) , rounded2(1,1):rounded2(2,1), : );
    end
    
    hsv_target_L = rgb2hsv(target_L);
    
    target_L_H = hsv_target_L(:,:,1) * 180;   % An array of all 'H' values in rgb target_L image, mult. by 180 due to 0-1 range for H
    target_L_S = hsv_target_L(:,:,2) * 255;   % An array of all 'S' values in rgb target_L image, mult. by 255
    target_L_V = hsv_target_L(:,:,3) * 255;   % An array of all 'V' values in rgb target_R image, mult. by 255
    
    target_L_H_avg = mean(mean(target_L_H));   % Average of all target_R_r values
    target_L_S_avg = mean(mean(target_L_S));   % Average of all target_R_g values
    target_L_V_avg = mean(mean(target_L_V));   % Average of all target_R_b values
    
    target_L_HSL_thresh = [target_L_H_avg target_L_S_avg target_L_V_avg]   % Rounded average rgb values in one single array (GRIP)
    
    
    hsv_target_R = rgb2hsv(target_R);
    
    target_R_H = hsv_target_R(:,:,1) * 180;   % An array of all 'H' values in rgb target_L image, mult. by 180 due to 0-1 range for H
    target_R_S = hsv_target_R(:,:,2) * 255;   % An array of all 'S' values in rgb target_L image, mult. by 255
    target_R_V = hsv_target_R(:,:,3) * 255;   % An array of all 'V' values in rgb target_R image, mult. by 255
    
    target_R_H_avg = mean(mean(target_R_H));   % Average of all target_R_r values
    target_R_S_avg = mean(mean(target_R_S));   % Average of all target_R_g values
    target_R_V_avg = mean(mean(target_R_V));   % Average of all target_R_b values
    
    target_R_HSL_thresh = [target_R_H_avg target_R_S_avg target_R_V_avg]   % Rounded average rgb values in one single array
    
    
    target_H    = [ target_R_H(:) ; target_L_H(:) ];
    H_min       = min(target_H);
    H_max       = max(target_H);
    disp([ 'H_range = ' num2str(H_min) ' - ' num2str(H_max) ]);
    
    
    target_S    = [ target_R_S(:) ; target_L_S(:) ];
    S_min       = min(target_S);
    S_max       = max(target_S);
    disp([ 'S_range = ' num2str(S_min) ' - ' num2str(S_max) ]);
    
    target_V_val    = [ target_R_V(:) ; target_L_V(:) ];
    V_min       = min(target_V_val);
    V_max       = max(target_V_val);
    disp([ 'V_range = ' num2str(V_min) ' - ' num2str(V_max) ]);
    
    H_threshold  = max( 0, min( 180, [ (H_min - H_margin) (H_max + H_margin) ]))
    S_threshold  = max( 0, min( 255, [ (S_min - S_margin) (S_max + S_margin) ]))
    V_threshold  = max( 0, min( 255, [ (V_min - V_margin) (V_max + V_margin) ]))
    
    target_L_h = size(target_L, 1);
    target_L_w = size(target_L, 2);    
    target_R_h = size(target_R, 1);
    target_R_w = size(target_R, 2);
    final_h    = min( final_h, min(target_L_h, target_R_h));
    final_w    = min( final_w, min(target_L_w, target_R_w));
    
    if img == 1
        final_H_threshold = H_threshold;
        final_S_threshold = S_threshold;
        final_V_threshold = V_threshold;
    else
        final_H_threshold(1) = min(final_H_threshold(1), H_threshold(1))
        final_H_threshold(2) = max(final_H_threshold(2), H_threshold(2));
        final_S_threshold(1) = min(final_S_threshold(1), S_threshold(1))
        final_S_threshold(2) = max(final_S_threshold(2), S_threshold(2));
        final_V_threshold(1) = min(final_V_threshold(1), V_threshold(1))
        final_V_threshold(2) = max(final_V_threshold(2), V_threshold(2));
    end
end
%% Store calibration data in structure
Calib.Hmin      = final_H_threshold(1);
Calib.Hmax      = final_H_threshold(2);
Calib.Smin      = final_S_threshold(1);
Calib.Smax      = final_S_threshold(2);
Calib.Vmin      = final_V_threshold(1);
Calib.Vmax      = final_V_threshold(2);
Calib.MinHeight = final_h;
Calib.MinWidth  = final_w;
Calib.MinRatio  = 0.1;
Calib.MaxRatio  = 0.5;

make_Vision_Calibration_h_file_v002(Calib);

%% Plot figure windows
figure
imshow(target_L);
figure
imshow(target_R);