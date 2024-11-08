function [success, html] = gendochtml(topic,pagetitle,helptitle,varargin)
%HELPHTML MATLAB file help displayed in a window
%   HELPHTML TOPIC generates the help text for the specified TOPIC in the
%   form of an html formatted string.  Links are created to functions 
%   referenced in the 'See Also' line of the help text.
%


if nargout
    success = true;
end

% Make sure that we can support the helpwin command on this platform.
if ~usejava('mwt')
    error(message('MATLAB:doc:UnsupportedPlatform',upper(mfilename)));
end

% This function has to support the old helpwin syntax, which
% provides for three disparate cases.
% Case 1: HELPWIN TOPIC, to display function or topic help (same as HELP function).
% Case 2: HELPWIN(HELP_STR,TITLE), to display an arbitrary help string.
% Case 3: HELPWIN({TITLE1 HELP_STR1;TITLE2 HELP_STR2;...},PAGE)
% to display multi-page help.

% Is this multi-page help text?  (case 3 above)
if nargin>0 && iscell(topic) && (size(topic,2) > 1)
    if nargin < 2
        pagetitle = '';
        helptitle = '';
    elseif nargin < 3
        helptitle = pagetitle;
    end
    if nargin > 3
        displayOption = varargin{1};
    else
        displayOption = '';
    end
    handle_multipage_help(topic,pagetitle,helptitle,displayOption)
    return;
end

if (nargin == 0)
    html = vHelp2html;
elseif (nargin == 1)
    html = vHelp2html(topic);
elseif (nargin > 1)
    if nargin > 3
        helpCommandOption = varargin{1};
    else
        helpCommandOption = '-helpwin';
    end
    html = vHelp2html(topic,pagetitle,helpCommandOption);
end

if isempty(html) && nargout
    success = false;
else
    displayFile(['text://' html]);
end

function handle_multipage_help(topic,pagetitle,helptitle,displayOption)
if iscell(topic) && numel(topic) == 2
    % This isn't really multipage help.  Use a different helpwin syntax.
    helpwin(topic{2},topic{1});
    return;
end

% Alternate between multiple temp files (starting with matlabTemp1.html).
% Multiple files are needed because a single file won't refresh properly,
% and to allow for a certain level of "Back" operations in the browser.
% You have to pass the browser a new file name each time to get it to reload.
dirName = char(com.mathworks.services.Prefs.getPropertyDirectory);
currFile = char(com.mathworks.mlservices.MLHelpServices.getCurrentLocation);
tempNum = regexp(currFile,'matlabTemp(\d).html','tokens','once');
if ~isempty(tempNum)
    tempNum = rem(str2double(tempNum),5) + 1;
else
    tempNum = 1;
end
fileName = fullfile(dirName,['matlabTemp' num2str(tempNum) '.html']);

% Workaround for ICE bug involving named anchors
fnPos = findstr(fileName, 'matlabTemp');
shortName = fileName(fnPos:length(fileName));

% Starting building HTML output, beginning with optional title banner.
% At top put header with links to each page.
% Only show header if more than one page.
dom = com.mathworks.xml.XMLUtils.createDocument('help-info');
dom.getDomConfig.setParameter('cdata-sections',true);
addTextNode(dom,dom.getDocumentElement,'title',helptitle);
addTextNode(dom,dom.getDocumentElement,'filename',shortName);

helpdir = fileparts(mfilename('fullpath'));
helpdir = ['file:///' strrep(helpdir,'\','/')];
addTextNode(dom,dom.getDocumentElement,'helptools-dir',helpdir);

if ~strcmp(helptitle,pagetitle)
    addTextNode(dom,dom.getDocumentElement,'pagetitle',pagetitle);
end
numpages = size(topic,1);
for x = 1:numpages
    pgtitle = char(topic(x,1));
    helpNode = help2xml(dom,pgtitle,pgtitle,'-helpwin');
    addAttribute(dom,helpNode,'id',num2str(x));
end

addTextNode(dom,dom.getDocumentElement,'default-topics-text',getString(message('MATLAB:helpwin:sprintf_DefaultTopics')));
addTextNode(dom,dom.getDocumentElement,'help-command-option','helpwin');
xslfile = fullfile(fileparts(mfilename('fullpath')),'private','helpwin.xsl');
html = xslt(dom,xslfile,'-tostring');

for x = 1:numpages
    helpstr = char(topic(x,2));
    if (size(helpstr,1) > 1)
        % Convert a 2-D char array to 1-D.
        c = cellstr(helpstr);
        helpstr = sprintf('%s\n',c{:});
    end
       
    html = regexprep(html,['\s*<!--\s*helptext ' num2str(x) '\s*-->'], sprintf('\n\n%s',regexptranslate('escape',helpstr)));
end

if strcmp(displayOption,'-nofile')
    displayFile(['text://' html]);
else
    fid = fopen(fileName,'w');
    fprintf(fid,'%s', html);
    fclose(fid);

    % Get the index for the requested page from the cell array.
    % Display the appropriate page.
    ind = strmatch(pagetitle,topic(:,1),'exact');
    if isempty(ind), ind = 1; end
    if ind == 1
        displayFile(fileName);
    else
        namedAnchor = sprintf('#topic%u',ind);
        displayFile([fileName namedAnchor ]);
    end
end

function addTextNode(dom,parent,name,text)
child = dom.createElement(name);
child.appendChild(dom.createTextNode(text));
parent.appendChild(child);

function addAttribute(dom,elt,name,text)
att = dom.createAttribute(name);
att.appendChild(dom.createTextNode(text));
elt.getAttributes.setNamedItem(att);


%------------------------------------------
% Helper function that displays the HTML file in the web browser.
function displayFile(htmlFile)
%web(htmlFile, '-helpbrowser');






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
