function [lineFound, lineOffset, lineAngle, endAhead, redCount] = detectLine(R, G, B)
%#codegen
% Red line detection for MathWorks Minidrone Competition (PX4 model).
% Inputs : R, G, B  - uint8 camera channels, 120x160 each
% Outputs: lineFound  - true if enough red pixels are visible
%          lineOffset - sideways position of the line under the drone,
%                       -1 (far left) .. 0 (centre) .. +1 (far right)
%          lineAngle  - direction the line heads, radians, 0 = straight
%                       ahead, positive = bends to the right (image right)
%          endAhead   - true if the line is visible but stops before the
%                       top band (end of track is coming)
%          redCount   - number of red pixels (for tuning / scopes)
%
% ASSUMPTION: top of the image = forward. Check this in the Video Viewer
% and flip the bands if it is not.

% ---- tunable parameters ----
redThresh  = 50;   % how "red" a pixel must be
minPixels  = 30;   % min red pixels to say the line is visible
minBandPix = 5;    % min red pixels in a band to trust its centre

% ---- 1. red mask (same idea as the example's blue mask) ----
r = double(R); g = double(G); b = double(B);
redness = r - g/2 - b/2;
mask = redness > redThresh;

[H, W] = size(mask);          % 120 x 160
redCount = sum(mask(:));
lineFound = redCount > minPixels;

% ---- 2. split image into far (top) and near (middle) bands ----
farRows = 1:floor(H/3);                    % top third  = ahead
midRows = floor(H/3)+1:floor(2*H/3);       % middle third = under drone
cols = 1:W;

farCols = sum(mask(farRows, :), 1);        % red count per column
midCols = sum(mask(midRows, :), 1);
farN = sum(farCols);
midN = sum(midCols);

% column centre of the red pixels in each band (fixed-size, codegen safe)
cFar = W/2;  cMid = W/2;
if farN > minBandPix
    cFar = sum(farCols .* cols) / farN;
end
if midN > minBandPix
    cMid = sum(midCols .* cols) / midN;
end

% ---- 3. outputs ----
lineOffset = 0;
lineAngle  = 0;
endAhead   = false;

if lineFound
    lineOffset = (cMid - W/2) / (W/2);

    if farN > minBandPix && midN > minBandPix
        rowFar = mean(farRows);
        rowMid = mean(midRows);
        lineAngle = atan2(cFar - cMid, rowMid - rowFar);
    end

    endAhead = (farN <= minBandPix) && (midN > minBandPix);
end
end
