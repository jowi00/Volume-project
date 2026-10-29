clear; clc; close all; tic;

%% Folder Configuration
leftImgDir  = 'Project_2/raw/eval/camera_left_ir_image_raw';
rightImgDir = 'Project_2/raw/eval/camera_right_ir_image_raw';
leftInfoDir = 'Project_2/raw/eval/camera_left_ir_camera_info';
rightInfoDir= 'Project_2/raw/eval/camera_right_ir_camera_info';
outputDir   = 'generated_depth_maps';

if ~exist(outputDir, 'dir'), mkdir(outputDir); end

%% 1. Extract Geometric Parameters from Camera Info
infoFilesLeft  = dir(fullfile(leftInfoDir, '*.txt'));
infoFilesRight = dir(fullfile(rightInfoDir, '*.txt'));

leftTxtPath  = fullfile(leftInfoDir, infoFilesLeft(1).name);
rightTxtPath = fullfile(rightInfoDir, infoFilesRight(1).name);

[focalLength, baseline, imageSize] = parse_camera_yaml(leftTxtPath, rightTxtPath);

%% 2. Pair Images Using Nearest Timestamp Matching
imgFilesLeft  = dir(fullfile(leftImgDir, '*.png'));
imgFilesRight = dir(fullfile(rightImgDir, '*.png'));

% Extract numerical timestamps in nanoseconds
tsLeft = zeros(length(imgFilesLeft), 1);
for i = 1:length(imgFilesLeft)
    tok = extractBetween(imgFilesLeft(i).name, 'left_ir_image_', '.png');
    tsLeft(i) = str2double(tok{1});
end

tsRight = zeros(length(imgFilesRight), 1);
for i = 1:length(imgFilesRight)
    tok = extractBetween(imgFilesRight(i).name, 'right_ir_image_', '.png');
    tsRight(i) = str2double(tok{1});
end

% Maximum allowed time difference threshold: 50 milliseconds (50 * 10^6 ns)
maxTimeDiffNs = 50e6; 

% Matching table: [idxLeft, idxRight, diffNs]
pairs = [];
for i = 1:length(tsLeft)
    [minDiff, minIdx] = min(abs(tsRight - tsLeft(i)));
    if minDiff <= maxTimeDiffNs
        pairs = [pairs; i, minIdx, minDiff]; %#ok<AGROW>
    end
end

fprintf('Found %d matching pairs out of %d left images.\n', size(pairs, 1), length(imgFilesLeft));

%% 3. Batch Process IR Image Pairs
for k = 1:size(pairs, 1)
    idxL = pairs(k, 1);
    idxR = pairs(k, 2);
    
    leftName  = imgFilesLeft(idxL).name;
    rightName = imgFilesRight(idxR).name;
    
    timestampStr = extractBetween(leftName, 'left_ir_image_', '.png');
    
    leftPath  = fullfile(leftImgDir, leftName);
    rightPath = fullfile(rightImgDir, rightName);
    
    % Read IR images
    I_left  = imread(leftPath);
    I_right = imread(rightPath);
    
    % Convert to grayscale if necessary
    if size(I_left, 3) == 3, I_left = rgb2gray(I_left); end
    if size(I_right, 3) == 3, I_right = rgb2gray(I_right); end

    %% 4. Calculate Disparity Map using disparitySGM
    disparityRange = [0, 64]; 
    
    disparityMap = disparitySGM(I_left, I_right, ...
        'DisparityRange', disparityRange, ...
        'UniquenessThreshold', 10);

    %% 5. Convert Disparity to Depth: Z = (f * B) / d
    % Replace invalid disparity values (<= 0) with NaN
    disparityMap(disparityMap <= 0) = NaN;
    
    depthMap = (focalLength * baseline) ./ disparityMap; % Depth in meters

    %% 6. Clean and Filter Noise
    % 2D Median Filter
    depthMapFiltered = medfilt2(depthMap, [5 5], 'symmetric');
    
    % Distance threshold (filter out background beyond 4 meters and closer than 0.3 meters)
    depthMapFiltered(depthMapFiltered < 0.3 | depthMapFiltered > 4.0) = NaN;

    %% 7. Save Depth Maps (.mat)
    saveMatPath = fullfile(outputDir, sprintf('depth_%s.mat', timestampStr{1}));
    save(saveMatPath, 'depthMapFiltered', 'depthMap');
    
    fprintf('Processed image [%d/%d]: %s (Time difference: %.2f ms)\n', ...
        k, size(pairs, 1), timestampStr{1}, pairs(k, 3) / 1e6);
end

%% 8. Processing Time
elapsed_time = toc;
disp(['Processing time: ' num2str(elapsed_time) ' sec']);