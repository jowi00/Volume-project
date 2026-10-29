function [cleaned_mask, depth_masked, eval_score] = segment_extinguisher(RGB, depth_img)
    % SEGMENT_EXTINGUISHER Performs 
    % - HSV segmentation and 
    % - Morphological cleaning
    % - Evaluates mask quality based on depth consistency
    
    % --- 1. HSV Segmentation ---
    HSV = rgb2hsv(RGB);
    H = HSV(:,:,1);
    S = HSV(:,:,2);
    V = HSV(:,:,3);
    
    % Threshold for the red color of the fire extinguisher
    hue_mask = (H <= 0.02) | (H >= 0.94);
    sat_mask = (S >= 0.40);
    val_mask = (V >= 0.20);
    
    raw_mask = hue_mask & sat_mask & val_mask;
    
    % --- 2. Morphological post-processing ---
    se_close = strel('disk', 7);
    closed_mask = imclose(raw_mask, se_close);
    
    filled_mask = imfill(closed_mask, 'holes');
    
    se_open = strel('disk', 3);
    cleaned_mask = imopen(filled_mask, se_open);
    
    % Noise Reduction
    cleaned_mask = bwareafilt(cleaned_mask, [500, Inf]);
    
    % --- 3. Application to the Depth Map ---
    if size(cleaned_mask, 1) ~= size(depth_img, 1) || size(cleaned_mask, 2) ~= size(depth_img, 2)
        mask_resized = imresize(cleaned_mask, [size(depth_img, 1), size(depth_img, 2)], 'nearest');
    else
        mask_resized = cleaned_mask;
    end
    
    depth_masked = depth_img;
    if ndims(depth_img) == 3
        depth_masked(repmat(~mask_resized, [1, 1, size(depth_img, 3)])) = 0;
    else
        depth_masked(~mask_resized) = 0;
    end
    
    % --- 4. Mask Evaluation (3D Consistency / Depth Variability) ---
    valid_depth_values = double(depth_img(mask_resized & (depth_img > 0)));
    
    if isempty(valid_depth_values)
        % if empty mask
        eval_score = 0; 
    else
        % Calculating the standard deviation of the depth values in the masked region
        std_depth = std(valid_depth_values);
        mean_depth = mean(valid_depth_values);
    
        % Coefficient of variation (the smaller it is, the more homogeneous the object is)
        cv = std_depth / (mean_depth + eps);
    
        % Conversion to a score expressed as a percentage (normalized)
        % A CV <= 0.05 (5% variation) results in a score close to 100%
        eval_score = max(0, min(100, (1 - cv / 0.20) * 100));
    end
end