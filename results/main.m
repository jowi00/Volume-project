%% MAIN.M - COMPLETE RECONSTRUCTION AND VOLUME ESTIMATION PIPELINE
% This script executes the entire processing pipeline:
%   1. Depth map generation
%   2. HSV segmentation and depth masking
%   3. Video tracking with bounding box and frame-by-frame volume display
%   4. Global 3D ICP reconstruction and volume estimation

clear; clc; close all;

%% 0. CHECK SCRIPT AVAILABILITY
requiredScripts = {
    'parse_camera_yaml.m', ...
    'segment_extinguisher.m', ...
    'generate_depth_maps.m', ...
    'segmentation.m', ...
    'reconstruction.m', ...
    'generate_bounding_box_video.m'
    };

missingScripts = {};
for i = 1:length(requiredScripts)
    if exist(requiredScripts{i}, 'file') ~= 2
        missingScripts{end+1} = requiredScripts{i}; %#ok<SAGROW>
    end
end

if ~isempty(missingScripts)
    error('Fichiers manquants dans le répertoire courant : %s', strjoin(missingScripts, ', '));
end

%% 1. STEP 1: DEPTH MAP ESTIMATION AND GENERATION
try
    run('generate_depth_maps.m');
    fprintf('-> Étape 1 terminée avec succès.\n\n');
catch ME
    error('Erreur lors de l''exécution de generate_depth_maps.m :\n%s', ME.message);
end

%% 2. STEP 2: SEGMENTATION AND BINARY MASK APPLICATION
try
    run('segmentation.m');
    fprintf('-> Étape 2 terminée avec succès.\n\n');
catch ME
    error('Erreur lors de l''exécution de segmentation.m :\n%s', ME.message);
end

%% 3. STEP 3: VIDEO GENERATION WITH BOUNDING BOX AND VOLUME
try
    run('generate_bounding_box_video.m');
    fprintf('-> Étape 4 terminée avec succès.\n\n');
catch ME
    error('Erreur lors de l''exécution de generate_bounding_box_video.m :\n%s', ME.message);
end

%% STEP 4: GLOBAL 3D RECONSTRUCTION AND VOLUME ESTIMATION
try
    run('reconstruction.m');
    fprintf('-> Étape 4 terminée avec succès.\n\n');
catch ME
    error('Erreur lors de l''exécution de reconstruction.m :\n%s', ME.message);
end

%% PIPELINE COMPLETION
clc;
elapsed_time = toc;
fprintf('Finish!\n');