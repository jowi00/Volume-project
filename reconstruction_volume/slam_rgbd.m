
clc
clear
close all
rng(0)

scriptFolder = fileparts(mfilename('fullpath'));
root = scriptFolder;

% Support scripts stored either in the project root or in a code subfolder
if ~isfolder(fullfile(root, 'raw')) && ~isfolder(fullfile(root, 'results'))
    parentFolder = fileparts(scriptFolder);
    if isfolder(fullfile(parentFolder, 'raw')) || isfolder(fullfile(parentFolder, 'results'))
        root = parentFolder;
    end
end

% Check that RGB-D SLAM is available
if exist('rgbdvslam', 'class') ~= 8
    error('rgbdvslam is not available. Check Navigation Toolbox and the MATLAB version.')
end

% Configuration
maxFrames = 85;
maxSync = 0.030;
minDepth = 0.5;
maxDepth = 6;
pixelStep = 2;
voxelMap = 0.025;

% Calibration
fx = 306.000244140625;
fy = 306.1123352050781;
cx = 318.4753112792969;
cy = 201.36949157714844;
Dcolor = [-1.2477725744247437 0.8747861981391907 -9.421713184565306e-05 -0.00014916047803126276 -0.2381284087896347 -1.2307056188583374 0.8520383238792419 -0.2296648770570755];

intrinsics = cameraIntrinsics([fx fy], [cx cy], [400 640]);

% RGB and depth files
rgbFolder = fullfile(root, 'raw', 'eval', 'camera_color_image_raw');
depthFolder = fullfile(root, 'raw', 'eval', 'camera_depth_image_raw');

rgbFiles = dir(fullfile(rgbFolder, '*.png'));
depthFiles = dir(fullfile(depthFolder, '*.png'));

[~, order] = sort({rgbFiles.name});
rgbFiles = rgbFiles(order);

[~, order] = sort({depthFiles.name});
depthFiles = depthFiles(order);

rgbTimes = zeros(numel(rgbFiles), 1);
depthTimes = zeros(numel(depthFiles), 1);

for i = 1:numel(rgbFiles)
    token = regexp(rgbFiles(i).name, '_(\d+)\.png$', 'tokens', 'once');
    rgbTimes(i) = str2double(token{1});
end

for i = 1:numel(depthFiles)
    token = regexp(depthFiles(i).name, '_(\d+)\.png$', 'tokens', 'once');
    depthTimes(i) = str2double(token{1});
end

% Synchronize the streams without reusing RGB images
rgbIdx = zeros(0,1);
depthIdx = zeros(0,1);
syncErrors = zeros(0,1);
usedRGB = false(numel(rgbFiles),1);

for i = 1:numel(depthFiles)
    [dt, j] = min(abs(rgbTimes - depthTimes(i)));
    dt = dt * 1e-9;

    if dt <= maxSync && ~usedRGB(j)
        rgbIdx(end+1,1) = j;
        depthIdx(end+1,1) = i;
        syncErrors(end+1,1) = dt;
        usedRGB(j) = true;
    end
end

if isempty(rgbIdx)
    error('No synchronized RGB-D pairs were found.')
end

n = min(maxFrames, numel(rgbIdx));
rgbIdx = rgbIdx(1:n);
depthIdx = depthIdx(1:n);
syncErrors = syncErrors(1:n);

fprintf('========== SYNCHRONIZATION ==========\n')
fprintf('Available RGB frames: %d\n', numel(rgbFiles))
fprintf('Available depth frames: %d\n', numel(depthFiles))
fprintf('Selected pairs: %d\n', n)
fprintf('Median timestamp difference: %.3f ms\n\n', median(syncErrors) * 1000)

% Create the RGB-D SLAM system
slam = rgbdvslam(intrinsics, 1000, 'DepthRange', [minDepth maxDepth], 'MaxNumPoints', 2000, 'SkipMaxFrames', 1, 'TrackFeatureRange', [10 250]);

fprintf('========== RGB-D SLAM ==========\n')

added = 0;

for i = 1:n
    rgb = imread(fullfile(rgbFiles(rgbIdx(i)).folder, rgbFiles(rgbIdx(i)).name));
    rgb = rectificar_rgb_racional(rgb, fx, fy, cx, cy, Dcolor);
    depth = imread(fullfile(depthFiles(depthIdx(i)).folder, depthFiles(depthIdx(i)).name));

    if ~isequal(size(rgb,1), size(depth,1)) || ~isequal(size(rgb,2), size(depth,2))
        error('The RGB and depth resolutions do not match in frame %d.', i)
    end

    addFrame(slam, rgb, depth);
    added = i;

    fprintf('[%d/%d] RGB-D frame submitted\n', i, n)

    status = checkStatus(slam);

    if status == uint8(0)
        fprintf('Tracking lost. Stopping the sequence.\n')
        break
    end
end

% Wait for all queued frames to be processed
while ~isDone(slam)
    pause(0.1)
end

finalStatus = checkStatus(slam);
fprintf('Final tracking status: %d\n', uint8(finalStatus))

% Extract the optimized camera poses
[camPoses, keyFrameIDs] = poses(slam);

fprintf('\n========== TRACKING ==========\n')
fprintf('Submitted frames: %d\n', added)
fprintf('Keyframes with an estimated pose: %d\n', numel(keyFrameIDs))

if isempty(keyFrameIDs)
    error('SLAM could not estimate any camera pose.')
end

% Display the SLAM result
figure('Name', 'RGB-D SLAM trajectory', 'Color', 'w')
plot(slam)
title('RGB-D SLAM - Camera trajectory')

% Dense reconstruction from the keyframes
mapXYZ = zeros(0,3,'single');
mapRGB = zeros(0,3,'uint8');
cameraPositions = zeros(numel(keyFrameIDs),3);

fprintf('\n========== DENSE RECONSTRUCTION ==========\n')

for k = 1:numel(keyFrameIDs)
    i = double(keyFrameIDs(k));

    rgb = imread(fullfile(rgbFiles(rgbIdx(i)).folder, rgbFiles(rgbIdx(i)).name));
    rgb = rectificar_rgb_racional(rgb, fx, fy, cx, cy, Dcolor);
    depth = imread(fullfile(depthFiles(depthIdx(i)).folder, depthFiles(depthIdx(i)).name));

    [h,w,~] = size(rgb);

    rows = 1:pixelStep:h;
    cols = 1:pixelStep:w;
    [u,v] = meshgrid(cols, rows);

    d = double(depth(rows,cols)) / 1000;
    sampledRGB = rgb(rows,cols,:);

    valid = isfinite(d) & d > minDepth & d < maxDepth;

    x = (u - 1 - cx) .* d / fx;
    y = (v - 1 - cy) .* d / fy;

    xyz = [x(valid) y(valid) d(valid)];

    color = reshape(sampledRGB, [], 3);
    color = color(valid(:),:);

    % Transform points from camera coordinates to world coordinates
    worldXYZ = transformPointsForward(camPoses(k), xyz);

    mapXYZ = [mapXYZ; single(worldXYZ)];
    mapRGB = [mapRGB; color];

    cameraPositions(k,:) = camPoses(k).Translation;

    fprintf('Keyframe %d: %d points\n', i, size(xyz,1))
end

% Create and downsample the global point cloud
mapFull = pointCloud(mapXYZ, 'Color', mapRGB);
mapPreview = pcdownsample(mapFull, 'gridAverage', voxelMap);

% Display the scene using the original RGB colours
figure('Name', 'RGB-D Global Map', 'Color', 'w')
pcshow(mapPreview)
hold on
plot3(cameraPositions(:,1), cameraPositions(:,2), cameraPositions(:,3), 'w-o', 'LineWidth', 1.5, 'MarkerFaceColor', 'y')
title(sprintf('RGB-D Global Map - %d keyframes', numel(keyFrameIDs)))
xlabel('X (m)')
ylabel('Y (m)')
zlabel('Z (m)')
axis equal
rotate3d on

% Save the results
output = fullfile(root, 'results', 'rgbd_slam');

if ~exist(output, 'dir')
    mkdir(output)
end

pcwrite(mapFull, fullfile(output, 'mapa_rgbd_completo.ply'), 'Encoding', 'binary');
pcwrite(mapPreview, fullfile(output, 'mapa_rgbd_preview.ply'), 'Encoding', 'binary');

save(fullfile(output, 'slam_poses.mat'), 'camPoses', 'keyFrameIDs', 'cameraPositions', 'rgbIdx', 'depthIdx', 'syncErrors');

fprintf('\n========== RESULTS ==========\n')
fprintf('Keyframes: %d\n', numel(keyFrameIDs))
fprintf('Accumulated points: %d\n', mapFull.Count)
fprintf('Points in preview cloud: %d\n', mapPreview.Count)
fprintf('Saved to: %s\n', output)
