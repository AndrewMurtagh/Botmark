clear; close all; clc;

STEP_PAUSE = false;
NUMFRAMES = 38;
REDTHRESHOLD = 0.85; 
GREENTHRESHOLD = 0.75;
BLUETHRESHOLD = 0.8;
ROWS = 360;
COLS = 640;

for frameinc=0:NUMFRAMES-1
    str = strcat(['../Data/object_tracking_images/', num2str(frameinc, '%04i'), '.png']);
    frame = imread(str);
    
    if STEP_PAUSE
        imshow(frame);
        title('Original');
        waitforbuttonpress;
    end
    
    
    redonly = frame(:,:,1);
    if STEP_PAUSE
        imshow(redonly);
        title('Red Plane');
        waitforbuttonpress;
    end
    redbinary = im2bw(redonly, REDTHRESHOLD);
    if STEP_PAUSE
        imshow(redbinary);
        title('Red Binary');
        waitforbuttonpress;
    end
    
    
    greenonly = frame(:,:,2);
    if STEP_PAUSE
        imshow(greenonly);
        title('Green Plane');
        waitforbuttonpress;
    end
    greenbinary = im2bw(greenonly, REDTHRESHOLD);
    if STEP_PAUSE
        imshow(greenbinary);
        title('Green Binary');
        waitforbuttonpress;
    end
    
    
    blueonly = frame(:,:,3);
    if STEP_PAUSE
        imshow(blueonly);
        title('Blue Plane');
        waitforbuttonpress;
    end
    bluebinary = im2bw(blueonly, BLUETHRESHOLD);
    if STEP_PAUSE
        imshow(bluebinary);
        title('Blue Binary');
        waitforbuttonpress;
    end
    
    
    combinedbinary = zeros(240, 320);
    for i=1:ROWS
        for j=1:COLS
           %combinedbinary(i,j) = greenbinary(i,j) + redbinary(i,j) + bluebinary(i,j);
           combinedbinary(i,j) = greenbinary(i,j) || redbinary(i,j) || bluebinary(i,j);
        end
    end

    %combinedbinary = medfilt2(combinedbinary);
    for i=2:ROWS-1
        for j=2:COLS-1
            
        filter=[combinedbinary(i-1,j-1),combinedbinary(i-1,j),combinedbinary(i-1,j+1),...
            combinedbinary(i,j-1),combinedbinary(i,j),combinedbinary(i,j+1),...
            combinedbinary(i+1,j-1),combinedbinary(i+1,j),combinedbinary(i+1,j+1)];
        
        combinedbinary(i,j)=median(filter);
        end
    end
    
    blobarea = 0;
    sumi=0;
    sumj=0;
     for i=1:ROWS
        for j=1:COLS
           blobarea = blobarea +  double(combinedbinary(i,j));
           sumi = sumi + i*double(combinedbinary(i,j));
           sumj = sumj + j*double(combinedbinary(i,j));
        end
    end
    centroidi = sumi/blobarea;
    centroidj = sumj/blobarea;


    imshow(combinedbinary);
    hold on;
    plot(int32(centroidj), int32(centroidi), 'ro');
    waitforbuttonpress;
    
    
end




%{
v = VideoReader('tracking.mp4');
num=0;
for i=1:6:223
    str = strcat('../Data/trackingimages/', sprintf('%04i',num), '.png');
    disp(str);
    num = num + 1;
    vidFrame = read(v, i);
    vidFrame = imresize(vidFrame, [360 640]);
   % [X,map] = rgb2ind(vidFrame, 256);
    %imwrite(X, map, str, 'bitdepth', 8);
    imwrite(vidFrame, str);
    
end
%}




