function checkSystemRequirements()
    %64bit Matlab and Windows is required
    assert(strcmp(computer('arch'),'win64'),'Error: ScanImage requires Matlab 64bit on Windows 64bit. This computer architecture is %s.',computer('arch'));
    
    %minimum required SI version is Matlab 2017a
    assert(~verLessThan('matlab','9.2.0'),'Error: ScanImage requires Matlab 2017a or later. This Matlab version is %s.',regexprep(getfield(ver('matlab'),'Release'),'[\(\)]',''));
    
    %check for multiple scanimage versions on path
    list = what('scanimage');
    mask = cellfun(@(m)any(strcmpi(m,'SI.m')),{list.m});
    numSI = sum(mask);
    
    if numSI > 1
        msgbox('Multiple ScanImage installations were found on the path.','Error','error');
        folders = strjoin({list(mask).path},'\n\t');
        error('Multiple ScanImage installations were found on the path:\n\t%s\nRemove the redundant instances from the path and restart Matlab.',folders);
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
