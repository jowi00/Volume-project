clear; clc; close all; tic;

%% 1. Path Configuration and Parameters
depthMaskedDir = 'depth_masked/';

depthFiles = dir(fullfile(depthMaskedDir, '*.png'));
numFrames  = length(depthFiles);

if numFrames == 0
    error('No images found in folder "%s".', depthMaskedDir);
end

depthScale = 1.0; 
gridStep   = 0.005; 

% Depth camera intrinsic parameters
fx = 525.0; fy = 525.0; cx = 319.5; cy = 239.5;
intrinsics = cameraIntrinsics([fx, fy], [cx, cy], [400, 640]);

%% 2. Automatic Search for the First VALID Frame
fprintf('Searching for the first frame containing the fire extinguisher...\n');

startIdx = 0;
ptCloudGlobal = pointCloud.empty;

for i = 1:numFrames
    imgPath = fullfile(depthMaskedDir, depthFiles(i).name);
    ptCloudTemp = processDepthFrame(imgPath, depthScale, intrinsics, gridStep);
    
    if ~isempty(ptCloudTemp) && ptCloudTemp.Count >= 100
        ptCloudGlobal = ptCloudTemp;
        startIdx = i;
        fprintf('-> First valid frame found: Frame %d/%d with %d points!\n\n', ...
            i, numFrames, ptCloudGlobal.Count);
        break;
    end
end

if isempty(ptCloudGlobal)
    error('Unable to load a valid frame.');
end

%% 3. Multi-View Alignment Loop (ICP)
frameStep = 3; 

fprintf('ICP alignment of subsequent views (step size: %d)...\n', frameStep);

for i = (startIdx + frameStep):frameStep:numFrames
    fprintf('Frame %d/%d (Global points: %d)... ', i, numFrames, ptCloudGlobal.Count);
    tic;
    
    imgPath = fullfile(depthMaskedDir, depthFiles(i).name);
    ptCloudCurrent = processDepthFrame(imgPath, depthScale, intrinsics, gridStep);
    
    if isempty(ptCloudCurrent) || ptCloudCurrent.Count < 50
        fprintf('Skipped (not enough points).\n');
        continue;
    end
    
    % ICP Alignment
    [tform, ~, rmse] = pcregistericp(ptCloudCurrent, ptCloudGlobal, ...
        'Metric', 'pointToPoint', ...
        'MaxIterations', 30, ...
        'Tolerance', [0.001, 0.25]);
    
    % Cloud Fusion
    ptCloudAligned = pctransform(ptCloudCurrent, tform);
    ptCloudGlobal  = pcmerge(ptCloudGlobal, ptCloudAligned, gridStep);
    
    tElapsed = toc;
    fprintf('OK (RMSE: %.4f m, Time: %.2f s)\n', rmse, tElapsed);
end

%% 4. Final Cleaning of the Merged Point Cloud
fprintf('\nFinal cleaning of the merged point cloud...\n');
ptCloudGlobalClean = pcdenoise(ptCloudGlobal, 'NumNeighbors', 20, 'Threshold', 1.0);

pts = double(ptCloudGlobalClean.Location);
validIdx = ~isnan(pts(:,1)) & ~isnan(pts(:,2)) & ~isnan(pts(:,3));
cleanPts = pts(validIdx, :);

%% 5. CALCULATE DIMENSIONS AND ANALYTICAL CYLINDRICAL VOLUME
fprintf('Calculating dimensions and cylindrical volume...\n');

dx = max(cleanPts(:,1)) - min(cleanPts(:,1));
dy = max(cleanPts(:,2)) - min(cleanPts(:,2));
dz = max(cleanPts(:,3)) - min(cleanPts(:,3));

dims = sort([dx, dy, dz], 'descend');
height_m = dims(1);

% Corrected diameter (nominal 17 cm if the visible side is incomplete)
visible_width = dims(2);
diameter_m    = max(visible_width, 0.17); 
radius_m      = diameter_m / 2;

% Volume of cylindrical model (+10% addition for dome and handle)
volCyl_m3 = pi * (radius_m^2) * height_m;
volCyl_tot_L = (volCyl_m3 * 1000.0) * 1.10; 

%% 6. GENERATE A 360° CYLINDER FOR ALPHA SHAPE
fprintf('Reconstructing 360° geometry for Alpha Shape...\n');

% 1. Find the main vertical axis of the fire extinguisher (largest dimension)
[~, mainAxisIdx] = max([dx, dy, dz]);

% Geometric center of the point cloud
x0 = mean(cleanPts(:,1));
y0 = mean(cleanPts(:,2));
z0 = mean(cleanPts(:,3));

% 2. Generate a complete 360° synthetic shell around the central axis
numAngles = 36;
numHeights = 50;
angles = linspace(0, 2*pi, numAngles);

if mainAxisIdx == 1 % Main axis along X
    hVals = linspace(min(cleanPts(:,1)), max(cleanPts(:,1)), numHeights);
    [A, H] = meshgrid(angles, hVals);
    ptsCyl = [H(:), y0 + radius_m*cos(A(:)), z0 + radius_m*sin(A(:))];
elseif mainAxisIdx == 2 % Main axis along Y
    hVals = linspace(min(cleanPts(:,2)), max(cleanPts(:,2)), numHeights);
    [A, H] = meshgrid(angles, hVals);
    ptsCyl = [x0 + radius_m*cos(A(:)), H(:), z0 + radius_m*sin(A(:))];
else % Main axis along Z
    hVals = linspace(min(cleanPts(:,3)), max(cleanPts(:,3)), numHeights);
    [A, H] = meshgrid(angles, hVals);
    ptsCyl = [x0 + radius_m*cos(A(:)), y0 + radius_m*sin(A(:)), H(:)];
end

% Combine actual point cloud with the 360° cylindrical structure
ptsComplete = [cleanPts; ptsCyl];

% 3. Alpha Shape on reconstructed 360° cylinder
alphaVal = 0.08; 
shp = alphaShape(ptsComplete(:,1), ptsComplete(:,2), ptsComplete(:,3), alphaVal);

volAlpha_m3 = volume(shp);
volAlpha_L  = volAlpha_m3 * 1000.0;

% Error calculation
volumeRef  = 10.0;
errAlpha   = (abs(volAlpha_L - volumeRef) / volumeRef) * 100;
errCyl     = (abs(volCyl_tot_L - volumeRef) / volumeRef) * 100;

%% 7. Display Results
fprintf('\n================ RECONSTRUCTION RESULTS ================\n');
fprintf('Measured Height (H)         : %.2f cm\n', height_m * 100);
fprintf('Corrected Diameter (D)      : %.2f cm\n', diameter_m * 100);
fprintf('---------------------------------------------------------\n');
fprintf('Volume (Alpha Shape 360°)   : %.2f Liters (Error: %.2f %%)\n', volAlpha_L, errAlpha);
fprintf('Volume (Cylindrical Model)  : %.2f Liters (Error: %.2f %%)\n', volCyl_tot_L, errCyl);
fprintf('Reference Volume            : %.2f Liters\n', volumeRef);
fprintf('=========================================================\n');

%% 8. Visualization
figure('Name', '360° Cylindrical 3D Reconstruction of Fire Extinguisher', 'Color', 'w');

% Subplot 1: ICP Point Cloud
subplot(1,2,1);
pcshow(ptCloudGlobalClean, 'MarkerSize', 25);
title('3D Point Cloud (ICP)');
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
grid on; axis equal;

% Subplot 2: 360° Alpha Shape
subplot(1,2,2);
plot(shp, 'FaceColor', [0.8 0.2 0.2], 'FaceAlpha', 0.7, 'EdgeColor', 'none');
camlight; lighting gouraud;
title(sprintf('AlphaShape 360°: %.2f L | Cyl: %.2f L', volAlpha_L, volCyl_tot_L));
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
grid on; axis equal;

% Corrected automatic bounds framing on ptsComplete
xBounds = [min(ptsComplete(:,1))-0.1, max(ptsComplete(:,1))+0.1];
yBounds = [min(ptsComplete(:,2))-0.1, max(ptsComplete(:,2))+0.1];
zBounds = [min(ptsComplete(:,3))-0.1, max(ptsComplete(:,3))+0.1];

subplot(1,2,1); xlim(xBounds); ylim(yBounds); zlim(zBounds);
subplot(1,2,2); xlim(xBounds); ylim(yBounds); zlim(zBounds);

%% 9. Processing Time
elapsed_time = toc;
disp(['Processing time: ' num2str(elapsed_time) ' sec']);

%% --- LOCAL FUNCTION FOR FRAME PROCESSING AND DYNAMIC FILTERING ---
function ptCloudClean = processDepthFrame(imgPath, depthScale, intrinsics, gridStep)
    depthImg = imread(imgPath);
    depthDouble = double(depthImg);
    
    if max(depthDouble(:)) == 0
        ptCloudClean = pointCloud.empty;
        return;
    end
    
    ptCloud = pcfromdepth(depthDouble, depthScale, intrinsics);
    pts = reshape(ptCloud.Location, [], 3);
    
    validIdx = ~isnan(pts(:,3)) & pts(:,3) > 0.3 & pts(:,3) < 4.0;
    ptsValid = pts(validIdx, :);
    
    if isempty(ptsValid)
        ptCloudClean = pointCloud.empty;
        return;
    end
    
    % Dynamic window around median depth
    zMedian = median(ptsValid(:,3));
    zMask   = abs(ptsValid(:,3) - zMedian) < 0.35; 
    ptsFiltered = ptsValid(zMask, :);
    
    if isempty(ptsFiltered)
        ptCloudClean = pointCloud.empty;
        return;
    end
    
    ptCloudFiltered = pointCloud(ptsFiltered);
    ptCloudDown     = pcdownsample(ptCloudFiltered, 'gridAverage', gridStep);
    
    if ptCloudDown.Count > 10
        ptCloudClean = pcdenoise(ptCloudDown, 'NumNeighbors', 15, 'Threshold', 1.0);
    else
        ptCloudClean = ptCloudDown;
    end
end