
clc
clear
close all

root = fileparts(mfilename('fullpath'));

% Usar las poses hibridas cuando esten disponibles
dataFolder = fullfile(root, 'results', 'rgbd_slam_hibrido');
useHybrid = isfile(fullfile(dataFolder, 'slam_poses.mat'));

if ~useHybrid
    dataFolder = fullfile(root, 'results', 'rgbd_slam');
end

load(fullfile(dataFolder, 'slam_poses.mat'), 'camPoses', 'keyFrameIDs', 'rgbIdx', 'depthIdx');
fprintf('Poses cargadas desde: %s\n\n', dataFolder)

% Configuracion
voxelSize = 0.04;
minViews = 2;
minDepth = 0.5;
maxDepth = 6;

% Calibracion
fx = 306.000244140625;
fy = 306.1123352050781;
cx = 318.4753112792969;
cy = 201.36949157714844;
Dcolor = [-1.2477725744247437 0.8747861981391907 -9.421713184565306e-05 -0.00014916047803126276 -0.2381284087896347 -1.2307056188583374 0.8520383238792419 -0.2296648770570755];

% Buscar imagenes
rgbFolder = fullfile(root, 'raw', 'eval', 'camera_color_image_raw');
depthFolder = fullfile(root, 'raw', 'eval', 'camera_depth_image_raw');

rgbFiles = dir(fullfile(rgbFolder, '*.png'));
depthFiles = dir(fullfile(depthFolder, '*.png'));

[~,idx] = sort({rgbFiles.name});
rgbFiles = rgbFiles(idx);

[~,idx] = sort({depthFiles.name});
depthFiles = depthFiles(idx);

% Acumular puntos en coordenadas globales
allXYZ = zeros(0,3,'single');
allRGB = zeros(0,3,'uint8');
allFrame = zeros(0,1,'uint16');

n = numel(keyFrameIDs);

fprintf('========== RGB-D FUSION ==========\n')
fprintf('Keyframes disponibles: %d\n', n)

for k = 1:n

    frame = double(keyFrameIDs(k));

    rgb = imread(fullfile(rgbFiles(rgbIdx(frame)).folder, rgbFiles(rgbIdx(frame)).name));
    rgb = rectificar_rgb_racional(rgb, fx, fy, cx, cy, Dcolor);
    depth = imread(fullfile(depthFiles(depthIdx(frame)).folder, depthFiles(depthIdx(frame)).name));

    [h,w] = size(depth);

    % Profundidad en metros
    z = double(depth) / 1000;

    % Reconstruir todos los pixeles
    [u,v] = meshgrid(1:w,1:h);

    valid = isfinite(z) & z > minDepth & z < maxDepth;

    x = (u - 1 - cx) .* z / fx;
    y = (v - 1 - cy) .* z / fy;

    xyz = [x(valid) y(valid) z(valid)];

    % Colores originales
    colors = reshape(rgb,[],3);
    colors = colors(valid(:),:);

    % Transformar al sistema global
    worldXYZ = transformPointsForward(camPoses(k),xyz);

    good = all(isfinite(worldXYZ),2);
    worldXYZ = worldXYZ(good,:);
    colors = colors(good,:);

    % Reducir redundancia dentro de cada captura
    cloud = pointCloud(single(worldXYZ),'Color',colors);
    cloud = pcdownsample(cloud,'gridAverage',voxelSize);

    xyzFrame = reshape(cloud.Location,[],3);
    rgbFrame = reshape(cloud.Color,[],3);

    allXYZ = [allXYZ; single(xyzFrame)];
    allRGB = [allRGB; rgbFrame];
    allFrame = [allFrame; repmat(uint16(k),size(xyzFrame,1),1)];

    fprintf('[%d/%d] Keyframe %d | %d puntos\n',k,n,frame,cloud.Count)
end

% Agrupar puntos en una cuadricula global
voxels = floor(double(allXYZ) / voxelSize);

[~,~,groups] = unique(voxels,'rows');
numVoxels = max(groups);

% Promediar geometria
xyzMean = zeros(numVoxels,3);

for c = 1:3
    xyzMean(:,c) = accumarray(groups,double(allXYZ(:,c)),[numVoxels 1],@mean);
end

% Promediar colores
rgbMean = zeros(numVoxels,3,'uint8');

for c = 1:3
    values = accumarray(groups,double(allRGB(:,c)),[numVoxels 1],@mean);
    rgbMean(:,c) = uint8(round(values));
end

% Contar cuantas capturas diferentes observan cada voxel
voxelFramePairs = unique([groups double(allFrame)],'rows');
observations = accumarray(voxelFramePairs(:,1),1,[numVoxels 1]);

% Mapa completo fusionado
mapAll = pointCloud(xyzMean,'Color',rgbMean);

% Mapa respaldado por varias capturas
stable = observations >= minViews;
mapStable = pointCloud(xyzMean(stable,:),'Color',rgbMean(stable,:));

% Estadisticas
fprintf('\n========== RESULTADOS ==========\n')
fprintf('Puntos antes de fusion: %d\n',size(allXYZ,1))
fprintf('Voxeles totales: %d\n',numVoxels)
fprintf('Voxeles observados una vez: %d\n',nnz(observations == 1))
fprintf('Voxeles observados al menos dos veces: %d\n',nnz(observations >= 2))
fprintf('Voxeles observados al menos tres veces: %d\n',nnz(observations >= 3))
fprintf('Cobertura multivista: %.1f %%\n',100*nnz(stable)/numVoxels)

% Visualizar comparacion
figure('Name','RGB-D Fusion Comparison','Color','w')
tiledlayout(1,2,'TileSpacing','compact')

nexttile
pcshow(mapAll)
title(sprintf('All voxels - %d',mapAll.Count))
xlabel('X (m)')
ylabel('Y (m)')
zlabel('Z (m)')
axis equal

nexttile
pcshow(mapStable)
title(sprintf('Multi-view voxels - %d',mapStable.Count))
xlabel('X (m)')
ylabel('Y (m)')
zlabel('Z (m)')
axis equal

% Visualizacion del mapa consistente
figure('Name','Consistent 3D Map','Color','w')
pcshow(mapStable)
title('Consistent RGB-D 3D Map')
xlabel('X (m)')
ylabel('Y (m)')
zlabel('Z (m)')
axis equal
rotate3d on

% Guardar resultados
if useHybrid
    output = fullfile(root,'results','fusion_rgbd_hibrida');
else
    output = fullfile(root,'results','fusion_rgbd');
end

if ~exist(output,'dir')
    mkdir(output)
end

pcwrite(mapAll,fullfile(output,'mapa_completo.ply'),'Encoding','binary');
pcwrite(mapStable,fullfile(output,'mapa_consistente.ply'),'Encoding','binary');

save(fullfile(output,'fusion_info.mat'),'observations','voxelSize','minViews','keyFrameIDs');

fprintf('\nResultados guardados en: %s\n',output)
