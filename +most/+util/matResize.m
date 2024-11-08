function outputMatrix = matResize(inputMatrix,scale)
%MATRESIZE Resizes a matrix.
%
% inputMatrix: the matrix data to resize.
% 
% scale: either a scalar factor by which to scale the matrix, or a 2-vector
% indicating [numRows,numCols] of the output matrix.
        
    if nargin < 2 || isempty(scale)
        error('Insufficient arguments');
    end

    if isscalar(scale)
        mNew = floor(size(inputMatrix,1)/scale);
        nNew = floor(size(inputMatrix,2)/scale);
    elseif length(scale) == 2
        mNew = scale(1);
        nNew = scale(2);
    else
        error('Invalid argument.');
    end
    
    [m,n,~] = size(inputMatrix);
    [X,Y] = meshgrid( (0:n-1)/(n-1), (0:m-1)/(m-1) );
    [XI,YI] = meshgrid( (0:nNew-1)/(nNew-1) , (0:mNew-1)/(mNew-1) );
    outputMatrix = zeros(mNew,nNew,size(inputMatrix,3));
    for i = 1:size(inputMatrix,3)
        outputMatrix(:,:,i) = max(interp2(X,Y,inputMatrix(:,:,i),XI,YI,'cubic',0),0.0); % max() clamps any values < 0.0
    end

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
