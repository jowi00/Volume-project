clear; clc; close all; tic;

%% 1. Directory Configuration
rawRgbDir     = fullfile('Project_2', 'raw', 'eval', 'camera_color_image_raw');
maskDir       = 'binary_masks';
depthMaskedDir= 'depth_masked';
outputVideo   = 'extinguisher_volume_tracking.mp4';

% Depth camera intrinsic parameters
fx = 525.0; fy = 525.0; cx = 319.5; cy = 239.5;
intrinsics = cameraIntrinsics([fx, fy], [cx, cy], [400, 640]);
gridStep   = 0.005;

%% 2. File Loading and Timestamp Extraction
rgbFiles   = dir(fullfile(rawRgbDir, 'camera_color_image_*.png'));
maskFiles  = dir(fullfile(maskDir, 'mask_camera_color_image_*.png'));
depthFiles = dir(fullfile(depthMaskedDir, 'depth_masked_camera_color_image_*.png'));

numRgb = length(rgbFiles);
if numRgb == 0
    error('No RGB images found in "%s".', rawRgbDir);
end

% Extract RGB timestamps
tsRgb = zeros(numRgb, 1);
for k = 1:numRgb
    tok = regexp(rgbFiles(k).name, 'camera_color_image_(\d+)\.png', 'tokens');
    if ~isempty(tok), tsRgb(k) = str2double(tok{1}{1}); end
end

% Extract mask timestamps
numMasks = length(maskFiles);
tsMasks  = zeros(numMasks, 1);
for k = 1:numMasks
    tok = regexp(maskFiles(k).name, 'mask_camera_color_image_(\d+)\.png', 'tokens');
    if ~isempty(tok), tsMasks(k) = str2double(tok{1}{1}); end
end

% Extract segmented depth map timestamps
numDepth = length(depthFiles);
tsDepth  = zeros(numDepth, 1);
for k = 1:numDepth
    tok = regexp(depthFiles(k).name, 'depth_masked_camera_color_image_(\d+)\.png', 'tokens');
    if ~isempty(tok), tsDepth(k) = str2double(tok{1}{1}); end
end

%% 3. Video Writer Initialization
videoWriter = VideoWriter(outputVideo, 'MPEG-4');
videoWriter.FrameRate = 10; % 10 frames per second
open(videoWriter);

fprintf('Video generation in progress for %d images...\n', numRgb);

%% 4. Processing Loop Across All 162 Images
for i = 1:numRgb
    rgbPath = fullfile(rawRgbDir, rgbFiles(i).name);
    I_rgb   = imread(rgbPath);
    t_curr  = tsRgb(i);
    
    % Find nearest matching mask (max allowed difference: 100 ms)
    [minDiffMask, idxMask] = min(abs(tsMasks - t_curr));
    hasMask = (numMasks > 0) && (minDiffMask < 100e6);
    
    bbox = [];
    volumeL = NaN;
    
    if hasMask
        maskPath = fullfile(maskDir, maskFiles(idxMask).name);
        mask = imread(maskPath);
        if ndims(mask) == 3, mask = mask(:,:,1); end
        maskBin = mask > 0;
        
        % Resize mask if necessary to match the RGB image dimensions
        if size(maskBin, 1) ~= size(I_rgb, 1) || size(maskBin, 2) ~= size(I_rgb, 2)
            maskBin = imresize(maskBin, [size(I_rgb, 1), size(I_rgb, 2)], 'nearest');
        end
        
        % Extract Bounding Box of the largest segmented object
        stats = regionprops(maskBin, 'BoundingBox', 'Area');
        if ~isempty(stats)
            [~, maxIdx] = max([stats.Area]);
            bbox = stats(maxIdx).BoundingBox; % [x, y, width, height]
        end
        
        % Match with segmented depth map to compute volume
        [minDiffDepth, idxDepth] = min(abs(tsDepth - t_curr));
        if (numDepth > 0) && (minDiffDepth < 100e6)
            depthPath  = fullfile(depthMaskedDir, depthFiles(idxDepth).name);
            depthImg   = imread(depthPath);
            depthDouble= double(depthImg);
            
            % Local volume estimation
            volumeL = estimateVolumeFromDepth(depthDouble, intrinsics, gridStep);
        end
    end
    
    % Overlay bounding box and text on the image
    if ~isempty(bbox)
        % Format volume label
        if ~isnan(volumeL) && volumeL > 0
            labelStr = sprintf('Extinguisher: %.2f L', volumeL);
        else
            labelStr = 'Extinguisher: Volume N/A';
        end
        
        % Graphical annotation (Bounding box + Text)
        I_annotated = insertObjectAnnotation(I_rgb, 'rectangle', bbox, labelStr, ...
            'LineWidth', 3, ...
            'Color', 'red', ...
            'TextColor', 'white', ...
            'FontSize', 14);
    else
        % If no fire extinguisher is segmented / detected
        I_annotated = insertText(I_rgb, [20 20], 'No extinguisher detected', ...
            'FontSize', 14, ...
            'BoxColor', 'black', ...
            'BoxOpacity', 0.6, ...
            'TextColor', 'yellow');
    end
    
    % Write frame to video
    writeVideo(videoWriter, I_annotated);
    
    if mod(i, 20) == 0 || i == numRgb
        fprintf('  Frame [%d/%d] processed.\n', i, numRgb);
    end
end

close(videoWriter);
fprintf('\nVideo generated successfully: %s\n', outputVideo);
elapsed_time = toc;
disp(['Processing time: ' num2str(elapsed_time) ' sec']);

%% --- LOCAL FUNCTION FOR PER-FRAME VOLUME ESTIMATION ---
function volLiters = estimateVolumeFromDepth(depthImg, intrinsics, gridStep)
    if max(depthImg(:)) == 0
        volLiters = NaN;
        return;
    end
    
    % Depth map scaling (Normalized uint16 or direct meters)
    if max(depthImg(:)) > 100
        depthDouble = depthImg / 65535.0;
    else
        depthDouble = depthImg;
    end
    
    ptCloud = pcfromdepth(depthDouble, 1.0, intrinsics);
    pts = reshape(ptCloud.Location, [], 3);
    
    % Filter valid points and apply a depth window around the median depth
    validIdx = ~isnan(pts(:,3)) & pts(:,3) > 0.3 & pts(:,3) < 4.0;
    ptsValid = pts(validIdx, :);
    
    if isempty(ptsValid) || size(ptsValid, 1) < 50
        volLiters = NaN;
        return;
    end
    
    zMedian  = median(ptsValid(:,3));
    ptsClean = ptsValid(abs(ptsValid(:,3) - zMedian) < 0.35, :);
    
    if size(ptsClean, 1) < 30
        volLiters = NaN;
        return;
    end
    
    % Calculate geometric dimensions of the point cloud
    dx = max(ptsClean(:,1)) - min(ptsClean(:,1));
    dy = max(ptsClean(:,2)) - min(ptsClean(:,2));
    dz = max(ptsClean(:,3)) - min(ptsClean(:,3));
    
    dims = sort([dx, dy, dz], 'descend');
    height_m      = dims(1);
    visible_width = dims(2);
    
    % Diameter correction and cylindrical model calculation
    diameter_m = max(visible_width, 0.17); 
    radius_m   = diameter_m / 2;
    
    volCyl_m3 = pi * (radius_m^2) * height_m;
    volLiters = (volCyl_m3 * 1000.0) * 1.10; % Account for top dome and handle (+10%)
end