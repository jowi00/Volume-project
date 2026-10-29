function [focalLength, baseline, imageSize] = parse_camera_yaml(leftTxtPath, rightTxtPath)
    % PARSE_CAMERA_YAML Parses text files from camera_info (YAML format)
    
    txtLeft = fileread(leftTxtPath);
    txtRight = fileread(rightTxtPath);
    
    % Extract image dimensions
    height = str2double(char(regexp(txtLeft, 'height:\s*(\d+)', 'tokens', 'once')));
    width = str2double(char(regexp(txtLeft, 'width:\s*(\d+)', 'tokens', 'once')));
    imageSize = [height, width];
    
    % Extract projection matrices P
    P_left = parseMatrixP(txtLeft);
    P_right = parseMatrixP(txtRight);
    
    % Focal length fx (element P(1,1))
    focalLength = P_left(1, 1);
    
    % Baseline calculation: P_right(1,4) equals fx * baseline (or -fx * baseline depending on convention)
    Tx = P_right(1, 4);
    baseline = abs(Tx / focalLength); % In meters
    
    fprintf('Focal length fx : %.4f px | Baseline B : %.4f m | Resolution : %dx%d\n', ...
        focalLength, baseline, width, height);
end

function P = parseMatrixP(str)
    % Extracts the 3x4 matrix 'P' as a numeric array
    tok = regexp(str, 'P:\s*\[([^\]]+)\]', 'tokens', 'once');
    vals = str2num(tok{1}); %#ok<ST2NM>
    P = reshape(vals, [4, 3])'; % Format 3x4
end