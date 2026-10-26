
clc
clear
close all
rng(0)

root = fileparts(mfilename('fullpath'));

% Comprobar que RGB-D SLAM esta disponible
if exist('rgbdvslam', 'class') ~= 8
    error('rgbdvslam no esta disponible. Comprueba Navigation Toolbox y la version de MATLAB.')
end

% Configuracion
maxFrames = 85;
maxSync = 0.030;
minDepth = 0.5;
maxDepth = 6;
pixelStep = 2;
voxelMap = 0.025;

% Calibracion
fx = 306.000244140625;
fy = 306.1123352050781;
cx = 318.4753112792969;
cy = 201.36949157714844;
Dcolor = [-1.2477725744247437 0.8747861981391907 -9.421713184565306e-05 -0.00014916047803126276 -0.2381284087896347 -1.2307056188583374 0.8520383238792419 -0.2296648770570755];

intrinsics = cameraIntrinsics([fx fy], [cx cy], [400 640]);

% Archivos RGB y profundidad
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

% Sincronizacion sin reutilizar imagenes RGB
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
    error('No hay parejas RGB-D sincronizadas')
end

n = min(maxFrames, numel(rgbIdx));
rgbIdx = rgbIdx(1:n);
depthIdx = depthIdx(1:n);
syncErrors = syncErrors(1:n);

fprintf('========== SYNCHRONIZATION ==========\n')
fprintf('RGB disponibles: %d\n', numel(rgbFiles))
fprintf('Depth disponibles: %d\n', numel(depthFiles))
fprintf('Parejas seleccionadas: %d\n', n)
fprintf('Diferencia temporal mediana: %.3f ms\n\n', median(syncErrors) * 1000)

% Crear sistema RGB-D SLAM
slam = rgbdvslam(intrinsics, 1000, 'DepthRange', [minDepth maxDepth], 'MaxNumPoints', 2000, 'SkipMaxFrames', 1, 'TrackFeatureRange', [10 250]);

fprintf('========== RGB-D SLAM ==========\n')

added = 0;

for i = 1:n
    rgb = imread(fullfile(rgbFiles(rgbIdx(i)).folder, rgbFiles(rgbIdx(i)).name));
    rgb = rectificar_rgb_racional(rgb, fx, fy, cx, cy, Dcolor);
    depth = imread(fullfile(depthFiles(depthIdx(i)).folder, depthFiles(depthIdx(i)).name));

    if ~isequal(size(rgb,1), size(depth,1)) || ~isequal(size(rgb,2), size(depth,2))
        error('Las resoluciones no coinciden en la captura %d', i)
    end

    addFrame(slam, rgb, depth);
    added = i;

    fprintf('[%d/%d] RGB-D enviado\n', i, n)

    status = checkStatus(slam);

    if status == uint8(0)
        fprintf('Tracking perdido. Se detiene la secuencia.\n')
        break
    end
end

% Completar el procesamiento
while ~isDone(slam)
    pause(0.1)
end

finalStatus = checkStatus(slam);
fprintf('Estado final del tracking: %d\n', uint8(finalStatus))

% Extraer poses optimizadas
[camPoses, keyFrameIDs] = poses(slam);

fprintf('\n========== TRACKING ==========\n')
fprintf('Capturas enviadas: %d\n', added)
fprintf('Keyframes con pose: %d\n', numel(keyFrameIDs))

if isempty(keyFrameIDs)
    error('SLAM no ha podido estimar ninguna pose')
end

% Mostrar resultado SLAM
figure('Name', 'RGB-D SLAM trajectory', 'Color', 'w')
plot(slam)
title('RGB-D SLAM - Camera trajectory')

% Reconstruccion densa usando los keyframes
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

    % Transformar puntos desde camara a mundo
    worldXYZ = transformPointsForward(camPoses(k), xyz);

    mapXYZ = [mapXYZ; single(worldXYZ)];
    mapRGB = [mapRGB; color];

    cameraPositions(k,:) = camPoses(k).Translation;

    fprintf('Keyframe %d: %d puntos\n', i, size(xyz,1))
end

% Crear y reducir nube global
mapFull = pointCloud(mapXYZ, 'Color', mapRGB);
mapPreview = pcdownsample(mapFull, 'gridAverage', voxelMap);

% Mostrar escena con colores reales
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

% Guardar resultados
output = fullfile(root, 'results', 'rgbd_slam');

if ~exist(output, 'dir')
    mkdir(output)
end

pcwrite(mapFull, fullfile(output, 'mapa_rgbd_completo.ply'), 'Encoding', 'binary');
pcwrite(mapPreview, fullfile(output, 'mapa_rgbd_preview.ply'), 'Encoding', 'binary');

save(fullfile(output, 'slam_poses.mat'), 'camPoses', 'keyFrameIDs', 'cameraPositions', 'rgbIdx', 'depthIdx', 'syncErrors');

fprintf('\n========== RESULTS ==========\n')
fprintf('Keyframes: %d\n', numel(keyFrameIDs))
fprintf('Puntos acumulados: %d\n', mapFull.Count)
fprintf('Puntos en preview: %d\n', mapPreview.Count)
fprintf('Guardado en: %s\n', output)
