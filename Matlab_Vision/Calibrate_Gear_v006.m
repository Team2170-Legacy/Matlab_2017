%
% Calibrate_Gear.m
% Written by Jacob Krucinski on 2/16/17

H_margin = 5;
S_margin = 5;
L_margin = 10;

final_H_treshold = [0 0];
final_S_treshold = [0 0];
final_L_treshold = [0 0];


% % img_001 = imread('Image_001.png');
% % % imshow(img_001);    % RGB Image
% % imshow(uint8(255) - img_001);
% % hsv_img = rgb2hsv(img_001);
% % 
% % clc
% disp('Click in the upper left and then the lower right of the Left gear target reflective tape...');
% rect1 = ginput(2)
% disp('Click in the upper left and then the lower right of the Left gear target reflective tape...');
% rect2 = ginput(2)
% % % rect1 = [108.0000 161.0000;
% % %   122.0000  216.0000];
% % %
% % % rect2 = [204.0000  164.0000;
% % %   216.0000  211.0000];
% % %
% 
% rounded1 = round(rect1);
% rounded2 = round(rect2);
% 
% % doesn't work img_001 has y-pixels first, x second target_L = img_001(rounded1(1,1):rounded1(2,1), rounded1(1,2):rounded1(2,2));
% target_L = img_001( rounded1(1,2):rounded1(2,2) , rounded1(1,1):rounded1(2,1), : );
% target_R = img_001( rounded2(1,2):rounded2(2,2) , rounded2(1,1):rounded2(2,1), : );

for img=1:3
    if img == 1
        img_001 = imread('Seq_1_001.png');
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
        img_002 = imread('Seq_1_002.png');
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
        img_003 = imread('Seq_1_003.png');
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
    
    target_L_H = (hsv_target_L(:,:,1) * 360) / 2;   % An array of all 'H' values in rgb target_L image, mult. by 359 due to 0-1 range for H
    target_L_S = hsv_target_L(:,:,2) * 255;   % An array of all 'S' values in rgb target_L image, mult. by 100 for percent
    target_L_L = hsv_target_L(:,:,3) * 100;   % An array of all 'V' values in rgb target_R image, mult. by 100 for percent
    
    target_L_H_avg = mean(mean(target_L_H));   % Average of all target_R_r values
    target_L_S_avg = mean(mean(target_L_S));   % Average of all target_R_g values
    target_L_L_avg = mean(mean(target_L_L));   % Average of all target_R_b values
    
    target_L_HSL_thresh = [target_L_H_avg target_L_S_avg target_L_L_avg]   % Rounded average rgb values in one single array (GRIP)
    
    
    hsv_target_R = rgb2hsv(target_R);
    
    target_R_H = (hsv_target_R(:,:,1) * 360) / 2;   % An array of all 'H' values in rgb target_L image, mult. by 359 due to 0-1 range for H
    target_R_S = hsv_target_R(:,:,2) * 255;   % An array of all 'S' values in rgb target_L image, mult. by 100 for percent
    target_R_L = hsv_target_R(:,:,3) * 100;   % An array of all 'V' values in rgb target_R image, mult. by 100 for percent
    
    target_R_H_avg = mean(mean(target_R_H));   % Average of all target_R_r values
    target_R_S_avg = mean(mean(target_R_S));   % Average of all target_R_g values
    target_R_L_avg = mean(mean(target_R_L));   % Average of all target_R_b values
    
    target_R_HSL_thresh = [target_R_H_avg target_R_S_avg target_R_L_avg]   % Rounded average rgb values in one single array
    
    
    target_H    = [ target_R_H(:) ; target_L_H(:) ];
    H_min       = min(target_H);
    H_max       = max(target_H);
    disp([ 'H_range = ' num2str(H_min) ' - ' num2str(H_max) ]);
    
    
    target_S    = [ target_R_S(:) ; target_L_S(:) ];
    S_min       = min(target_S);
    S_max       = max(target_S);
    disp([ 'S_range = ' num2str(S_min) ' - ' num2str(S_max) ]);
    
    target_L_val    = [ target_R_L(:) ; target_L_L(:) ];
    L_min       = min(target_L_val);
    L_max       = max(target_L_val);
    disp([ 'L_range = ' num2str(L_min) ' - ' num2str(L_max) ]);
    
    H_threshold  = max( 0, min( 180, [ (H_min - H_margin) (H_max + H_margin) ]))
    S_threshold  = max( 0, min( 255, [ (S_min - S_margin) (S_max + S_margin) ]))
    L_threshold  = max( 0, min( 255, [ ((L_min + 3) - L_margin) (L_max + L_margin) ]))
    
    target_L_h = size(target_L, 1);
    target_L_w = size(target_L, 2);    
    target_R_h = size(target_R, 1);
    target_R_w = size(target_R, 2);
    
    if img == 1
        final_H_threshold = H_threshold;
        final_S_threshold = S_threshold;
        final_L_threshold = L_threshold;
    else
        final_H_threshold(1) = min(final_H_threshold(1), H_threshold(1))
        final_H_threshold(2) = max(final_H_threshold(2), H_threshold(2));
        final_S_threshold(1) = min(final_S_threshold(1), S_threshold(1))
        final_S_threshold(2) = max(final_S_threshold(2), S_threshold(2));
        final_L_threshold(1) = min(final_L_threshold(1), L_threshold(1))
        final_L_threshold(2) = max(final_L_threshold(2), L_threshold(2));
    end
end

%% Plot figure windows
figure
imshow(target_L);
figure
imshow(target_R);