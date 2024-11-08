% this code was developed by Marius Pachitariu and Carsen Stringer as part of the software package Suite2p

% computes registration offset of data from mean image ops.mimg
% subpixel registration is kriging interpolation
% inputs:
%%% data = [ly, lx, nFrames]
%%% ops = options
%%% removeMean - if 1, then subtract mean offsets and realign to mean offset
%%% (for pre-registration alignment)
% outputs:
%%% offsets = [nFrames, 2]
%%% corr    = [nFrames]
%%% regdata = [ly, lx, nFrames] (only computed if removeMean is not empty!)

function [dv,corr] = regoffKriging(data, ops, removeMean)

[~,~,packageName] = most.idioms.getFunctionInfo();
import([packageName '.*']);

refImg = ops.mimg;
subpixel = getOr(ops, {'subPixel' 'SubPixel'}, 10); % subpixel factor
useGPU = getOr(ops, 'useGPU', false);
phaseCorrelation = getOr(ops, {'phaseCorrelation', 'PhaseCorrelation'}, true);
% maximum shift allowed
maxregshift = getOr(ops, 'maxregshift', round(.1*max(ops.Ly,ops.Lx))); 
% slope on taper mask preapplied to image. was 2, then 1.2
maskSlope   = getOr(ops, 'maskSlope', 2); 
% SD pixels of gaussian smoothing applied to correlation map (MOM likes .6)
smoothSigma = 1.15;


% if subpixel is still inf, threshold it for new method
if ops.kriging
    subpixel = min(10, subpixel);
end

[ly lx nFrames] = size(data);

% Taper mask
[ys, xs] = ndgrid(1:ly, 1:lx);
ys = abs(ys - mean(ys(:)));
xs = abs(xs - mean(xs(:)));
mY      = max(ys(:)) - 4;
mX      = max(xs(:)) - 4;
maskMul = single(1./(1 + exp((ys - mY)/maskSlope)) ./(1 + exp((xs - mX)/maskSlope)));
maskOffset = mean(refImg(:))*(1 - maskMul);

% Smoothing filter in frequency domain
hgx = exp(-(((0:lx-1) - fix(lx/2))/smoothSigma).^2);
hgy = exp(-(((0:ly-1) - fix(ly/2))/smoothSigma).^2);
hg = hgy'*hgx;
fhg = real(fftn(ifftshift(single(hg/sum(hg(:))))));

% fft of reference image 
eps0          = single(1e-10);
cfRefImg = conj(fftn(refImg));
if phaseCorrelation
    absRef   = abs(cfRefImg);
    cfRefImg = cfRefImg./(eps0 + absRef) .* fhg;
end

if useGPU
    batchSize = getBatchSize(ly*lx);
    eps0      = gpuArray(eps0);
    cfRefImg    = gpuArray(cfRefImg);
    maskMul = gpuArray(maskMul);
    maskOffset = gpuArray(maskOffset);    
else
    batchSize = 1000;
end

% allow max shifts +/- lcorr
lpad   = 3;
lcorr  = min(maxregshift, floor(min(ly,lx)/2)-lpad);

% only need a small kernel +/- lpad for smoothing
[x1,x2] = ndgrid([-lpad:lpad]);
xt = [x1(:) x2(:)]';
if useGPU
    xt = gpuArray(single(xt));
end

if ops.kriging
    % compute kernels for regression
    sigL     = .85; % kernel width in pixels
    Kx = kernelD(xt,xt,sigL*[1;1]);
    linds = [-lpad:1/subpixel:lpad];
    [x1,x2] = ndgrid(linds);
    xg = [x1(:) x2(:)]';
    if useGPU
        xg = gpuArray(single(xg));
    end
    Kg = kernelD(xg,xt,sigL*[1;1]);
    Kmat = Kg/Kx;
end

% loop over batches
dv = zeros(nFrames, 2);
corr = zeros(nFrames, 1);

nBatches = ceil(nFrames/batchSize);
for bi = 1:nBatches
    fi = ((bi - 1)*batchSize + 1):min(bi*batchSize, nFrames);
    
    if useGPU
        batchData = gpuArray(single(data(:,:,fi)));
    else
        batchData = single(data(:,:,fi));
    end
    
    corrMap = fft2(bsxfun(@plus, maskOffset, bsxfun(@times, maskMul, batchData)));
    
    %keyboard;
    if phaseCorrelation
        corrMap = bsxfun(@times, corrMap./(eps0 + abs(corrMap)), cfRefImg);
    else
        corrMap = bsxfun(@times, corrMap, cfRefImg);
    end
    
    % compute correlation matrix
    corrClip = real(ifft2(corrMap));
    corrClip = fftshift(fftshift(corrClip, 1), 2);
    corrClipSmooth = corrClip;
    
    %% subpixel registration
    if subpixel > 1
        % kriging subpixel
        % allow only +/- lcorr shifts
        cc0         = corrClipSmooth(floor(ly/2)+1+[-lcorr:lcorr],...
            floor(lx/2)+1+[-lcorr:lcorr],:);
        [cmax,ii]   = max(reshape(cc0, [], numel(fi)),[],1);
        
        [iy, ix] = ind2sub((2*lcorr+1) * [1 1], ii);
        
        dl       = single(-lpad:1:lpad);
        if useGPU
            dl   = gpuArray(dl);
            ccmat = gpuArray.zeros(numel(dl), numel(dl), numel(fi), 'single');
        else
            ccmat = zeros(numel(dl), numel(dl), numel(fi), 'single');
        end
        mxpt        = [iy(:)+floor(ly/2) ix(:)+floor(lx/2)] - lcorr;
        for j = 1:numel(fi)
            % matrix +/- lpad surrounding max point
            ccmat(:, :, j) = corrClip(mxpt(j,1)+dl, mxpt(j,2)+dl, j);
        end
        %
        ccmat = reshape(ccmat,[], numel(fi));
        if ops.kriging
            % regress onto subsampled grid
            ccb         = Kmat * ccmat;
            
            % find max of grid
            [cx,ix]     = max(ccb, [], 1);
            [ix11,ix21] = ind2sub(numel(linds)*[1 1],ix);
            mdpt        = floor(numel(linds)/2)+1;
            dv0         = bsxfun(@minus, ([ix11' ix21'] - mdpt)/subpixel + mxpt, ...
                [floor(ly/2) floor(lx/2)]) - 1;
        else
            yshift      = xt(1,:) * ccmat;
            xshift      = xt(2,:) * ccmat;
            dv0         = bsxfun(@rdivide, [yshift' xshift'], sum(ccmat, 1)') + mxpt;
            dv0         = bsxfun(@minus, dv0, [floor(ly/2) floor(lx/2)] + 1);
            if isfinite(subpixel)
                dv0 = round(dv0 * subpixel) / subpixel;
            end
            cx     = max(ccmat, [], 1);
        end
        dv(fi,:) = gather_try(dv0);
        corr(fi)  = gather_try(cx);
    % otherwise just take peak of matrix
    else
        cc0     = corrClipSmooth(floor(ly/2)+1+[-lcorr:lcorr],floor(lx/2)+1+[-lcorr:lcorr],:);
        [cmax,iy]  = max(cc0,[],1);
        [cx, ix]   = max(cmax,[],2);
        iy = reshape(iy(sub2ind([size(iy,2) size(iy,3)], ix(:), (1:size(iy,3))')),...
            1, 1, []);
        
        dv0 = [iy(:)-lcorr ix(:)-lcorr]-1;
        dv(fi,:)  = gather_try(dv0);
        corr(fi) = gather_try(cx(:));
    end
        
end

if nargin > 2 && removeMean
    dv = bsxfun(@minus, dv, mean(dv,1));
end






% ----------------------------------------------------------------------------
% Copyright (C) 2021 Vidrio Technologies, LLC
% 
% ScanImage (R) 2021 is software to be used under the purchased terms
% Code may be modified, but not redistributed without the permission
% of Vidrio Technologies, LLC
% 
% VIDRIO TECHNOLOGIES, LLC MAKES NO WARRANTIES, EXPRESS OR IMPLIED, WITH
% RESPECT TO THIS PRODUCT, AND EXPRESSLY DISCLAIMS ANY WARRANTY OF
% MERCHANTABILITY OR FITNESS FOR A PARTICULAR PURPOSE.
% IN NO CASE SHALL VIDRIO TECHNOLOGIES, LLC BE LIABLE TO ANYONE FOR ANY
% CONSEQUENTIAL OR INCIDENTAL DAMAGES, EXPRESS OR IMPLIED, OR UPON ANY OTHER
% BASIS OF LIABILITY WHATSOEVER, EVEN IF THE LOSS OR DAMAGE IS CAUSED BY
% VIDRIO TECHNOLOGIES, LLC'S OWN NEGLIGENCE OR FAULT.
% CONSEQUENTLY, VIDRIO TECHNOLOGIES, LLC SHALL HAVE NO LIABILITY FOR ANY
% PERSONAL INJURY, PROPERTY DAMAGE OR OTHER LOSS BASED ON THE USE OF THE
% PRODUCT IN COMBINATION WITH OR INTEGRATED INTO ANY OTHER INSTRUMENT OR
% DEVICE.  HOWEVER, IF VIDRIO TECHNOLOGIES, LLC IS HELD LIABLE, WHETHER
% DIRECTLY OR INDIRECTLY, FOR ANY LOSS OR DAMAGE ARISING, REGARDLESS OF CAUSE
% OR ORIGIN, VIDRIO TECHNOLOGIES, LLC's MAXIMUM LIABILITY SHALL NOT IN ANY
% CASE EXCEED THE PURCHASE PRICE OF THE PRODUCT WHICH SHALL BE THE COMPLETE
% AND EXCLUSIVE REMEDY AGAINST VIDRIO TECHNOLOGIES, LLC.
% ----------------------------------------------------------------------------
