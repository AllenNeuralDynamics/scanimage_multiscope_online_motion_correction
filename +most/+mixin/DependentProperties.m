classdef DependentProperties < handle
    %DEPENDENTPROPERTYCLASS
    
    % API changes from implementation in Model.m:
    %   * Call the registerDependentPropertiesForProperty method rather than specify
    %   dependencies in the propAttributes struct.
    %
    %   * The configuration is effectively the inverse of the original DependsOn
    %   struct field.  Each call to registerDependentPropertiesForProperty takes the
    %   source property that other properties are dependent on, and the list of
    %   properties that are dependent on that property.  This means that only one
    %   listener/event is required per property that has downstream dependent
    %   properties (rather than one per downstream property per source property),
    %   and is more consistent with other frameworks providing similar
    %   functionality.
    %
    %   * It is possible to register as dependent on another object's property and
    %   have this object's property(ies) be updated/trigger events as well.
    
    methods
        function lh = registerDependentPropertiesForProperty(self, sourcePropertyName, dependentPropertyNames, varargin)
            
            % Call the generic method for dependency on any object's property, using self as
            % the source object.
            lh = self.registerDependentPropertiesForPropertyOfObject(self, sourcePropertyName, dependentPropertyNames, varargin{:});
        end
        
        function lh = registerDependentPropertiesForPropertyOfObject(self, sourceObject, sourcePropertyName, dependentPropertyNames, notifyFcn)
            narginchk(4, 5);
            
            if nargin < 5
                notifyFcn = [];
            end
            
            assert(isprop(sourceObject, sourcePropertyName), 'DependentProperties:invalidproperty', '%s is not a recognized property for %s', sourcePropertyName, class(sourceObject));
            
            if ischar(dependentPropertyNames)
                dependentPropertyNames = {dependentPropertyNames};
            else
                % TODO validate cellstr.
            end
            
            for pdx = 1:numel(dependentPropertyNames)
                % TODO assert propertyNames{idx} is a property of the object and can be set.  It
                % should probably also be verified to have the Dependent attribute so that the
                % dummy set has no effect other than triggering a PostSet event.
            end
            
            lh = most.ErrorHandler.addCatchingListener(sourceObject, sourcePropertyName, 'PostSet', @(~, evt)znstPropertyChanged(self, dependentPropertyNames, notifyFcn));
            
            function znstPropertyChanged(targetObj, propertyNames, fcn)
                for idx = 1:numel(propertyNames)
                    if ~isempty(fcn)
                        feval(fcn, propertyNames{idx});
                    else
                        % This is an ugly hack because it is not appear possible to force or otherwise
                        % generate a PostSet property event.  That would remove the requirement for an
                        % empty set method for the dependent property.
                        %targetObj
                        propertyName=propertyNames{idx};
                        %targetObj.(propertyName) = nan;  
                        targetObj.(propertyName) = most.util.Nonvalue.The;  
                            % Setting to most.app.Nonvalue.The allows setter to be non-empty, as long as it rejects a most.app.Nonvalue value
                    end
                end
            end
        end
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
