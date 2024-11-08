function out=tokenize(line, varargin)
%TOKENIZE   - Delimits input based on white spaces.
% 	TOKENIZE(line) will take the input string line and use white spaces (except
% 	those at the trailign ends which are removed with DEBLANK) to delimit the
% 	string tokens in line.  The output is a cell array of strings, each one
% 	being one of the tokens from the input line.
% 
% 	Ex:  out=tokenize('  aa aaa aaa  ')
% 
% 	out = 
%   	 'aa'    'aaa'    'aaa'
%
%   TOKENIZE(line, delimiter) - Tokenize based on a custom delimiter. (TO022706D).
%
%  
% See also STRTOK, DEBLANK
%% NOTES
%   Note that this function regards text enclosed in single quotes to be a single token, including any speaces contained therein -- Vijay Iyer 1/30/09
%
%% CHANGES
% 	TPMOD1 (2/4/04) - Commented Function.
%   TPMOD2 (3/23/04) - Added delimiter option.
%   TO022706D: Optimization(s). Complete rewrite. -- Tim O'Connor 2/27/06
%% *******************************************************

% if isempty(varargin)
%     delimiterIndices = find(isspace(line));
% else
%     if length(varargin{1}) > 1
%         error('Delimiters may only be one character in length.');
%     end
%     delimiterIndices = find(line == varargin{1});
% end
% 
% delimiterIndices = [1 delimiterIndices(find(diff([1 delimiterIndices]) ~= 1))];
% 
% out = {};
% for i = 1 : 2 : length(delimiterIndices) - 1
%     out{i} = line(delimiterIndices(i) : delimiterIndices(i + 1) - 1);
% end
% 
% out{:}
% 
% return;

if nargin == 1
    delimiter = [9:13 32]; % White space characters;
else
    delimiter = varargin{1};
end

out={};
line=deblank(line);
line=fliplr(deblank(fliplr(line)));
while (length(line)>0)
	[token, line]=getToken(line,delimiter);
	if (length(token)>0)
		out{length(out)+1}=token;
	end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [token, remLine]=getToken(line,delimiter)
[token, remLine]=strtok(line,delimiter);
if length(token)==0
	return
end

if any(findstr(token,'''')) 
	while (length(remLine)>0) & (token(length(token))~='''') 
		[tok2, remLine]=strtok(remLine,delimiter);
		remLine=remLine(2:length(remLine));
		token=[token ' ' tok2];
	end
	if token(1)=='''' & token(length(token))==''''
		token=token(2:length(token)-1);
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
