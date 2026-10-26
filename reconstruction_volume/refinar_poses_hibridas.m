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

% Original SLAM poses
inputFolder = fullfile(root, 'results', 'rgbd_slam');
poseFile = fullfile(inputFolder, 'slam_poses.mat');

if ~isfile(poseFile)
    error(['slam_poses.mat was not found. Run slam_rgbd.m first. ' ...
           'Expected file: %s'], poseFile)
end

load(poseFile, 'camPoses', 'keyFrameIDs', 'rgbIdx', 'depthIdx', 'syncErrors');

% Configuration
minDepth = 0.5;
maxDepth = 6;
pixelStep = 3;
voxelICP = 0.05;
minInliers = 15;
minZones = 4;
maxRansacError = 0.06;
maxICPError = 0.08;
maxCorrectionTranslation = 0.50;
maxCorrectionRotation = 8;
maxDenseError = 0.08;
minDenseAgreement10 = 60;
minDenseCoverage = 40;

% Registered RGB-D calibration
fx = 306.000244140625;
fy = 306.1123352050781;
cx = 318.4753112792969;
cy = 201.36949157714844;
Dcolor = [-1.2477725744247437 0.8747861981391907 -9.421713184565306e-05 -0.00014916047803126276 -0.2381284087896347 -1.2307056188583374 0.8520383238792419 -0.2296648770570755];

% RGB and depth files
rgbFolder = fullfile(root, 'raw', 'eval', 'camera_color_image_raw');
depthFolder = fullfile(root, 'raw', 'eval', 'camera_depth_image_raw');

rgbFiles = dir(fullfile(rgbFolder, '*.png'));
depthFiles = dir(fullfile(depthFolder, '*.png'));

[~,order] = sort({rgbFiles.name});
rgbFiles = rgbFiles(order);

[~,order] = sort({depthFiles.name});
depthFiles = depthFiles(order);

n = numel(keyFrameIDs);

images = cell(n,1);
depths = cell(n,1);
features = cell(n,1);
locations = cell(n,1);
clouds = cell(n,1);

fprintf('========== HYBRID PREPARATION ==========\n')
fprintf('Keyframes: %d\n\n', n)

for k = 1:n
    frame = double(keyFrameIDs(k));

    images{k} = imread(fullfile(rgbFiles(rgbIdx(frame)).folder, rgbFiles(rgbIdx(frame)).name));
    images{k} = rectificar_rgb_racional(images{k}, fx, fy, cx, cy, Dcolor);
    depths{k} = imread(fullfile(depthFiles(depthIdx(frame)).folder, depthFiles(depthIdx(frame)).name));

    gray = im2gray(images{k});
    points = detectSIFTFeatures(gray);
    points = points.selectStrongest(2000);
    [features{k},validPoints] = extractFeatures(gray,points);
    locations{k} = double(validPoints.Location);

    clouds{k} = crear_nube(depths{k}, fx, fy, cx, cy, minDepth, maxDepth, pixelStep, voxelICP);

    fprintf('[%d/%d] Keyframe %d prepared\n', k, n, frame)
end

camPosesOriginal = camPoses;
camPosesHybrid = camPosesOriginal;

numMatches = zeros(n,1);
numInliers = zeros(n,1);
coverage = zeros(n,1);
ransacError = nan(n,1);
slamError = nan(n,1);
icpError = nan(n,1);
translationDiff = nan(n,1);
rotationDiff = nan(n,1);
denseSlamError = nan(n,1);
denseRansacError = nan(n,1);
denseSlamAgreement10 = nan(n,1);
denseRansacAgreement10 = nan(n,1);
denseSlamCoverage = nan(n,1);
denseRansacCoverage = nan(n,1);
corrected = false(n,1);
rejectedCorrection = false(n,1);
method = strings(n,1);
relativeUsed = repmat(eye(4),1,1,n);

fprintf('\n========== POSE REFINEMENT ==========\n')

for k = 2:n
    id1 = double(keyFrameIDs(k-1));
    id2 = double(keyFrameIDs(k));

    Tslam = camPosesOriginal(k-1).A \ camPosesOriginal(k).A;
    tformSLAM = rigidtform3d(Tslam);

    [tformRANSAC,status,pairMatches,pairInliers,pairCoverage,pairError,xyz1,xyz2,inlierMask] = estimar_ransac(features{k-1},features{k},locations{k-1},locations{k},depths{k-1},depths{k},fx,fy,cx,cy,minDepth,maxDepth);

    numMatches(k) = pairMatches;
    numInliers(k) = pairInliers;
    coverage(k) = pairCoverage;
    ransacError(k) = pairError;

    ransacReliable = status == 0 && pairInliers >= minInliers && pairCoverage >= minZones && pairError <= maxRansacError;
    denseImprovement = false;

    if pairInliers >= 6
        alignedSLAM = transformPointsForward(tformSLAM,xyz2(inlierMask,:));
        slamDistances = vecnorm(alignedSLAM-xyz1(inlierMask,:),2,2);
        slamError(k) = median(slamDistances);

        Test = tformRANSAC.A;
        translationDiff(k) = norm(Tslam(1:3,4)-Test(1:3,4));

        Rdiff = Test(1:3,1:3).' * Tslam(1:3,1:3);
        c = max(-1,min(1,(trace(Rdiff)-1)/2));
        rotationDiff(k) = acosd(c);
    end

    if ransacReliable
        [denseSlamError(k),denseSlamAgreement10(k),denseSlamCoverage(k)] = evaluar_geometria(depths{k-1},depths{k},tformSLAM,fx,fy,cx,cy,0.6,5,pixelStep);
        [denseRansacError(k),denseRansacAgreement10(k),denseRansacCoverage(k)] = evaluar_geometria(depths{k-1},depths{k},tformRANSAC,fx,fy,cx,cy,0.6,5,pixelStep);

        denseValid = denseRansacError(k) <= maxDenseError && denseRansacAgreement10(k) >= minDenseAgreement10 && denseRansacCoverage(k) >= minDenseCoverage;
        denseBetter = denseRansacError(k) <= 0.90*denseSlamError(k) || denseRansacAgreement10(k) >= denseSlamAgreement10(k)+5;
        denseImprovement = denseValid && denseBetter;
    end

    slamSuspicious = ransacReliable && (slamError(k) > max(0.08,2*pairError) || translationDiff(k) > 0.25 || rotationDiff(k) > 5);
    correctionPlausible = translationDiff(k) <= maxCorrectionTranslation && rotationDiff(k) <= maxCorrectionRotation;
    badSLAM = slamSuspicious && correctionPlausible && denseImprovement;

    chosen = tformSLAM;
    method(k) = "SLAM";

    if badSLAM
        chosen = tformRANSAC;
        method(k) = "RANSAC";

        try
            [tformICP,~,rmse] = pcregistericp(clouds{k},clouds{k-1},'Metric','pointToPlane','InitialTransform',tformRANSAC,'InlierDistance',0.12,'MaxIterations',60);
            icpError(k) = rmse;
            [denseICPError,denseICPAgreement10,denseICPCoverage] = evaluar_geometria(depths{k-1},depths{k},tformICP,fx,fy,cx,cy,0.6,5,pixelStep);

            if isfinite(rmse) && rmse <= maxICPError && denseICPError <= denseRansacError(k) && denseICPAgreement10 >= denseRansacAgreement10(k) && denseICPCoverage >= minDenseCoverage
                chosen = tformICP;
                method(k) = "RANSAC+ICP";
            end
        catch
            icpError(k) = NaN;
        end

        corrected(k) = true;
    elseif slamSuspicious
        rejectedCorrection(k) = true;
        method(k) = "SLAM (correction rejected)";
    end

    relativeUsed(:,:,k) = chosen.A;
    Tglobal = camPosesHybrid(k-1).A * chosen.A;
    camPosesHybrid(k) = rigidtform3d(Tglobal);

    fprintf('KF %2d -> %2d | 3D %3d | Inliers %3d | Regions %2d/16 | RANSAC %.3f | SLAM %.3f | Dense %.3f -> %.3f | %s\n', id1, id2, pairMatches, pairInliers, pairCoverage, pairError, slamError(k), denseSlamError(k), denseRansacError(k), method(k))
end

% Camera positions before and after pose correction
positionsOriginal = zeros(n,3);
positionsHybrid = zeros(n,3);

for k = 1:n
    positionsOriginal(k,:) = camPosesOriginal(k).Translation;
    positionsHybrid(k,:) = camPosesHybrid(k).Translation;
end

figure('Name','SLAM and hybrid trajectories','Color','w')
plot3(positionsOriginal(:,1),positionsOriginal(:,2),positionsOriginal(:,3),'-o','LineWidth',1.2)
hold on
plot3(positionsHybrid(:,1),positionsHybrid(:,2),positionsHybrid(:,3),'-o','LineWidth',1.5)
xlabel('X (m)')
ylabel('Y (m)')
zlabel('Z (m)')
legend('Original SLAM','Hybrid trajectory','Location','best')
title('Trajectory comparison')
axis equal
grid on
view(3)

% Save the corrected poses without overwriting the original poses
output = fullfile(root, 'results', 'rgbd_slam_hibrido');

if ~exist(output, 'dir')
    mkdir(output)
end

camPoses = camPosesHybrid;
cameraPositions = positionsHybrid;

save(fullfile(output, 'slam_poses.mat'), 'camPoses', 'keyFrameIDs', 'cameraPositions', 'rgbIdx', 'depthIdx', 'syncErrors');
save(fullfile(output, 'refinamiento.mat'), 'camPosesOriginal', 'camPosesHybrid', 'keyFrameIDs', 'numMatches', 'numInliers', 'coverage', 'ransacError', 'slamError', 'icpError', 'translationDiff', 'rotationDiff', 'denseSlamError', 'denseRansacError', 'denseSlamAgreement10', 'denseRansacAgreement10', 'denseSlamCoverage', 'denseRansacCoverage', 'corrected', 'rejectedCorrection', 'method', 'relativeUsed', 'positionsOriginal', 'positionsHybrid');

fprintf('\n========== HYBRID SUMMARY ==========\n')
fprintf('Corrected transformations: %d / %d\n', nnz(corrected), n-1)
fprintf('Corrections rejected by validation: %d\n', nnz(rejectedCorrection))
fprintf('Reliable RANSAC estimates: %d / %d\n', nnz(numInliers >= minInliers & coverage >= minZones & ransacError <= maxRansacError), n-1)
fprintf('Median SLAM error for evaluated pairs: %.3f m\n', median(slamError,'omitnan'))
fprintf('Median RANSAC error: %.3f m\n', median(ransacError,'omitnan'))
fprintf('Corrected poses saved to: %s\n', output)

function cloud = crear_nube(depth, fx, fy, cx, cy, minDepth, maxDepth, pixelStep, voxelSize)

[h,w] = size(depth);
rows = 1:pixelStep:h;
cols = 1:pixelStep:w;
[u,v] = meshgrid(cols,rows);

z = double(depth(rows,cols)) / 1000;
valid = isfinite(z) & z > minDepth & z < maxDepth;

x = (u-1-cx) .* z / fx;
y = (v-1-cy) .* z / fy;
xyz = [x(valid) y(valid) z(valid)];

cloud = pointCloud(single(xyz));
cloud = pcdownsample(cloud,'gridAverage',voxelSize);

end

function [errorMedian,agreement10,overlap] = evaluar_geometria(depth1, depth2, tform21, fx, fy, cx, cy, minDepth, maxDepth, pixelStep)

[h,w] = size(depth1);
rows = 1:pixelStep:h;
cols = 1:pixelStep:w;
[u,v] = meshgrid(cols,rows);

z1 = double(depth1(rows,cols)) / 1000;
valid = isfinite(z1) & z1 > minDepth & z1 < maxDepth;

x1 = (u-1-cx) .* z1 / fx;
y1 = (v-1-cy) .* z1 / fy;
xyz1 = [x1(valid) y1(valid) z1(valid)];

errorMedian = NaN;
agreement10 = NaN;
overlap = NaN;

if size(xyz1,1) < 100
    return
end

% tform21 transforms points from the second camera to the first camera.
xyz2 = transformPointsInverse(tform21,xyz1);
zPred = xyz2(:,3);
u2 = round(fx * xyz2(:,1) ./ zPred + cx + 1);
v2 = round(fy * xyz2(:,2) ./ zPred + cy + 1);

inside = isfinite(zPred) & zPred > minDepth & zPred < maxDepth;
inside = inside & isfinite(u2) & isfinite(v2) & u2 >= 1 & u2 <= w & v2 >= 1 & v2 <= h;

if nnz(inside) < 100
    return
end

indices = sub2ind([h w],v2(inside),u2(inside));
zMeasured = double(depth2(indices)) / 1000;
zPred = zPred(inside);
valid = isfinite(zMeasured) & zMeasured > minDepth & zMeasured < maxDepth;

zMeasured = zMeasured(valid);
zPred = zPred(valid);

if numel(zPred) < 100
    return
end

errors = abs(zPred-zMeasured);
errorMedian = median(errors);
agreement10 = 100*mean(errors < 0.10);
overlap = 100*numel(errors)/size(xyz1,1);

end

function [tform,status,numMatches,numInliers,coverage,errorMedian,xyz1,xyz2,inliers] = estimar_ransac(features1, features2, locations1, locations2, depth1, depth2, fx, fy, cx, cy, minDepth, maxDepth)

tform = rigidtform3d;
status = 1;
numMatches = 0;
numInliers = 0;
coverage = 0;
errorMedian = NaN;
xyz1 = zeros(0,3);
xyz2 = zeros(0,3);
inliers = false(0,1);

pairs = matchFeatures(features1,features2,'Unique',true,'MaxRatio',0.75,'MatchThreshold',25);

if size(pairs,1) < 12
    return
end

uv1 = locations1(pairs(:,1),:);
uv2 = locations2(pairs(:,2),:);

[h,w] = size(depth1);

u1 = round(uv1(:,1));
v1 = round(uv1(:,2));
u2 = round(uv2(:,1));
v2 = round(uv2(:,2));

valid = u1 >= 1 & u1 <= w & v1 >= 1 & v1 <= h;
valid = valid & u2 >= 1 & u2 <= w & v2 >= 1 & v2 <= h;

uv1 = uv1(valid,:);
uv2 = uv2(valid,:);
u1 = u1(valid);
v1 = v1(valid);
u2 = u2(valid);
v2 = v2(valid);

z1 = double(depth1(sub2ind([h w],v1,u1))) / 1000;
z2 = double(depth2(sub2ind([h w],v2,u2))) / 1000;

valid = z1 > minDepth & z1 < maxDepth & z2 > minDepth & z2 < maxDepth;

uv1 = uv1(valid,:);
uv2 = uv2(valid,:);
z1 = z1(valid);
z2 = z2(valid);

numMatches = numel(z1);

if numMatches < 12
    return
end

xyz1 = [(uv1(:,1)-1-cx).*z1/fx, (uv1(:,2)-1-cy).*z1/fy, z1];
xyz2 = [(uv2(:,1)-1-cx).*z2/fx, (uv2(:,2)-1-cy).*z2/fy, z2];

warningState = warning('off','vision:ransac:maxTrialsReached');
[tform,inliers,status] = estgeotform3d(xyz2,xyz1,'rigid','MaxDistance',0.08,'MaxNumTrials',5000,'Confidence',99.9);
warning(warningState)

numInliers = nnz(inliers);

if status ~= 0 || numInliers < 6
    return
end

gx = min(4,max(1,ceil(uv1(inliers,1)/(w/4))));
gy = min(4,max(1,ceil(uv1(inliers,2)/(h/4))));
coverage = numel(unique((gy-1)*4+gx));

aligned = transformPointsForward(tform,xyz2(inliers,:));
errors = vecnorm(aligned-xyz1(inliers,:),2,2);
errorMedian = median(errors);

end
