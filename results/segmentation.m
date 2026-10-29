clear; clc; close all; tic;

%% 1. Defining Input and Output Folders
rgb_dir = fullfile('Project_2', 'raw', 'eval', 'camera_color_image_raw');
depth_dir = 'generated_depth_maps'; % Dossier à la racine

mask_out_dir = 'binary_masks';
applied_out_dir = 'depth_masked';

if ~exist(mask_out_dir, 'dir'), mkdir(mask_out_dir); end
if ~exist(applied_out_dir, 'dir'), mkdir(applied_out_dir); end

%% 2. File Uploading and Time Synchronization
rgb_files = dir(fullfile(rgb_dir, 'camera_color_image_*.png'));
depth_files = dir(fullfile(depth_dir, 'depth_*.mat'));

if isempty(rgb_files) || isempty(depth_files)
    error('Aucune image ou fichier .mat trouvé dans les dossiers spécifiés.');
end

% Extracting Timestamps
rgb_timestamps = zeros(length(rgb_files), 1);
for k = 1:length(rgb_files)
    tokens = regexp(rgb_files(k).name, 'camera_color_image_(\d+)\.png', 'tokens');
    if ~isempty(tokens)
        rgb_timestamps(k) = str2double(tokens{1}{1});
    end
end

depth_timestamps = zeros(length(depth_files), 1);
for k = 1:length(depth_files)
    tokens = regexp(depth_files(k).name, 'depth_(\d+)\.mat', 'tokens');
    if ~isempty(tokens)
        depth_timestamps(k) = str2double(tokens{1}{1});
    end
end

num_images = length(rgb_files);

%% 3. Initializing the summary table
summary_table = table('Size', [num_images 3], ...
    'VariableTypes', {'string', 'string', 'double'}, ...
    'VariableNames', {'nom_de_l_image_rgb', 'mask_clean_associe', 'evaluation_score_pct'});

%% 4. Processing Loop
for i = 1:num_images
    rgb_filename = rgb_files(i).name;
    RGB = imread(fullfile(rgb_dir, rgb_filename));
    
    % Temporal Association of Depth (.mat file with closest timestamp)
    current_ts = rgb_timestamps(i);
    [~, idx_closest] = min(abs(depth_timestamps - current_ts));
    depth_filename = depth_files(idx_closest).name;
    
    % Loading the .mat file and extracting the depth matrix
    mat_data = load(fullfile(depth_dir, depth_filename));
    struct_fields = fieldnames(mat_data);
    depth_img = mat_data.(struct_fields{1});
    
    % Convert to uint16 / double if the matrix is normalized
    depth_img = double(depth_img);
    
    % Call to the segmentation and evaluation function
    [cleaned_mask, depth_masked, eval_score] = segment_extinguisher(RGB, depth_img);
    
    % Backup File Names
    mask_name = ['mask_' rgb_filename];
    depth_masked_name = ['depth_masked_' rgb_filename];
    
    % Saving Results (Masque binaire et Depth Map masquée au format PNG)
    imwrite(cleaned_mask, fullfile(mask_out_dir, mask_name));
    
    % Normalization/scaling for uint16 storage, if necessary
    if max(depth_masked(:)) <= 1.0
        imwrite(uint16(depth_masked * 65535), fullfile(applied_out_dir, depth_masked_name));
    else
        imwrite(uint16(depth_masked), fullfile(applied_out_dir, depth_masked_name));
    end
    
    % Filling in the table
    summary_table.nom_de_l_image_rgb(i) = string(rgb_filename);
    summary_table.mask_clean_associe(i) = string(mask_name);
    summary_table.evaluation_score_pct(i) = round(eval_score, 2);
end

% .CSV save
% writetable(summary_table, 'mask_evaluation.csv');

%% 5. Preparing data compatible with uitable (char & cell)
table_data = cell(num_images, 3);
for k = 1:num_images
    table_data{k, 1} = char(summary_table.nom_de_l_image_rgb(k));
    table_data{k, 2} = char(summary_table.mask_clean_associe(k));
    table_data{k, 3} = summary_table.evaluation_score_pct(k);
end

column_names = {'Image RGB', 'Masque Nettoyé Associé', 'Score Évaluation (%)'};

%% 6. Displaying results in a new GUI window
fig = figure('Name', 'Évaluation et Validation des Masques', ...
             'NumberTitle', 'off', ...
             'Position', [100, 100, 850, 500], ...
             'Resize', 'on');

uit = uitable(fig, ...
    'Data', table_data, ...
    'ColumnName', column_names, ...
    'ColumnWidth', {320, 270, 150}, ...
    'Units', 'normalized', ...
    'Position', [0.03, 0.05, 0.94, 0.90]);

%% 7. Processing time
elapsed_time = toc;
disp(['Processing time : ' num2str(elapsed_time) ' sec']);