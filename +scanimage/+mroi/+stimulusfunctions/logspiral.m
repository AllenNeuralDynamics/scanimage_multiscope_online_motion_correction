function [xx,yy] = logspiral(tt,varargin)
% logarithmic spiral stimulus

% the following line will be parsed by the ROI editor to present a list of
% options. should be in the format: parameter1 (comment), parameter2 (comment)
%% parameter options: revolutions (Number of revolutions), direction (Can be 'inward' or 'outward')

%% parse inputs
inputs = scanimage.mroi.util.parseInputs(varargin);

if ~isfield(inputs,'revolutions') || isempty(inputs.revolutions)
    inputs.revolutions = 5;
end

if ~isfield(inputs,'direction') || isempty(inputs.direction)
    inputs.direction = 'outward';
end

if ~isfield(inputs,'a') || isempty(inputs.a)
    inputs.a = 0;
end

tt = tt ./ tt(end); %normalize tt
switch inputs.direction
    case 'outward'
        % Nothing to do
        % tt = tt;
    case 'inward'
        tt = fliplr(tt);
    otherwise
        error('Unknown direction: %s',inputs.direction);
end

%% generate output
if inputs.a == 0;
    xx = tt .* sin(inputs.revolutions .* 2*pi .* tt);
    yy = tt .* cos(inputs.revolutions .* 2*pi .* tt);
else
    tt = tt-max(tt);
    xx = exp(inputs.a .* tt) .* sin(inputs.revolutions .* 2*pi .* tt);
    yy = exp(inputs.a .* tt) .* cos(inputs.revolutions .* 2*pi .* tt);
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
