% function hMotionEstimator = makeMotionEstimator(channel, filePath)
function hMotionEstimator = makeMotionEstimator(channel, target_z, zScan, filePath)
%     if nargin<2 || isempty(filePath)
    if nargin<4 || isempty(filePath)
        filePath = [];
    end

    im = readTiff(filePath); % read tiff
    
    zSpacing = 0.75; % in micron

    N = numel(im);
    zs = -(N-1)/2 : (N-1)/2;
    zs = zSpacing * zs;
    
    hSI = evalin('base','hSI'); % setting hSI

%     % scale image data
%     cLim = hSI.hChannels.channelLUT{channel};
%     imMax = max(cellfun(@(i)max(i,[],'all'),im));
%     imScaling = single(cLim(2))./imMax*8;
%     im = cellfun(@(i)i.*imScaling,im,'UniformOutput',false);

%     fastZPosition = hSI.hFastZ.hFastZs{1}.targetPosition; % setting the target z position
    fastZPosition = target_z;
    zs = zs + fastZPosition; % updating z values with target position offset
    zs = round(zs,2); % to prevent floating issue

    hROI = hSI.hRoiManager.currentRoiGroup.activeRois(1);
    % hROI = hROI.copy();
    % deleteMask = hROI.zs ~= target_z;
    % deleteIdxs = find(deleteMask);
    % hROI.removeById(deleteIdxs);
    % hROI.discretePlaneMode = false;

    hRoiData = scanimage.mroi.RoiData(); % setting RoiData object
    %hRoiData.hRoi = hSI.hRoiManager.currentRoiGroup.activeRois(1);
    hRoiData.hRoi = hROI;
    hRoiData.imageData = {im};
    hRoiData.channels = channel;
    hRoiData.zs = round(zs - target_z + zScan,2);

    hMotionEstimator = scanimage.components.motionEstimators.MariusMotionEstimator(hRoiData);
    hMotionEstimator.restrictZs = zScan;
%     hMotionEstimator = scanimage.components.motionEstimators.SimpleMotionEstimator(hRoiData); % note: runs on CPU, is slow!
end

function im = readTiff(filePath)
    if nargin<1 || isempty(filePath) % GUI for file selection
        title = 'Load ScanImage GUI Settings';
        filter = {'*.tif;*.tiff','Tiff files'};
        defaultName = '';

        [file,location] = uigetfile(filter,title,defaultName);

        if isequal(file,0)
            return % cancelled by user
        end

        filePath = fullfile(location,file);
    end

    try
        hTiff = Tiff(filePath);
        im = {};

        idx = 1;
        while true
            try
                im{idx} = single(hTiff.read())';
                hTiff.nextDirectory();
                idx = idx+1;
            catch ME
                break;
            end
        end

        hTiff.delete();
    catch ME
        hTiff.delete();
        ME.rethrow();
    end
end