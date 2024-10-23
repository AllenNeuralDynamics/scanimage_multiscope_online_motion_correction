classdef Beams
    properties
        beamIdx
        siBeamIdx
        sampleRateHz;
        powerFraction = 0;
        flybackBlanking = true;
        
        pzAdjust;
        Lz;
        pzFunction;
        pzLUT;
        pzReferenceZ;
        
        interlaceDecimation = 1;
        interlaceOffset = 0;
        
        powerFracToVoltageFunc;
        
        linePhase;          % [seconds]
        beamClockDelay;     % [seconds]
        beamClockExtend;    % [seconds]
        powerBoxes = [];
        
        includeFlybackLines = false;
        
        hDevice
    end
    
    properties (Dependent,SetAccess = private)
        powerFractionLimit
    end

    methods(Static)
        function obj = default
            obj=scanimage.mroi.scanners.Beams([]);
        end
    end
    
    methods
        function obj=Beams(hDevice)            
            obj.hDevice = hDevice;
        end
        
        function beamPath = generateBeamsPathStimulus(obj,path_FOV,scanfield,parkfunctiondetected,repetitionsInteger,durationPerRepetitionInt,durationPerRepetitionFrac,totalduration,maxPoints)
            if nargin < 9
                maxPoints = inf;
            end
            
            if isempty(scanfield.powerFractions)
                powerFrac = obj.powerFraction;
            elseif numel(scanfield.powerFractions) == 1
                powerFrac = scanfield.powerFractions;
            elseif numel(scanfield.powerFractions) >= obj.beamIdx                
                powerFrac = scanfield.powers(obj.beamIdx);
            end
            
            if isnan(powerFrac)
                powerFrac = obj.powerFraction;
            end
            
            [tf, idx] = ismember('poweredPause',scanfield.stimparams(1:2:end));
            poweredPause = tf && scanfield.stimparams{idx*2};
            
            isn = isfield(path_FOV, 'G') && all(all(isnan(path_FOV.G)));
            
            if (isn || parkfunctiondetected) && (~poweredPause)
                % detected output from the pause/park stimulus function. set
                % beam powers to zero
                numsamples = round(totalduration * obj.sampleRateHz);
                beamPath = zeros(numsamples,1);
            else                
                numsamples = round(durationPerRepetitionInt * obj.sampleRateHz);
                tt = linspace(0,(numsamples-1)/obj.sampleRateHz,min(numsamples,maxPoints));
                numsamples = length(tt); % recalculate in case maxPoints < numsamples
                
                beamPath = scanfield.beamsfcnhdl(tt,powerFrac,'actualDuration',durationPerRepetitionInt,'scanfield',scanfield);
                assert(numel(beamPath) == numsamples,...
                    ['Beams generation function ''%s'' returned incorrect number of samples:',...
                    'Expected: %d Returned: %d'],...
                    func2str(scanfield.beamsfcnhdl),numsamples,numel(beamPath));
                
                beamPath = beamPath(:);
                
                % apply repetitions
                beamPath = repmat(beamPath,repetitionsInteger,1);
                
                % fractional repetitions
                numsamples = round(durationPerRepetitionFrac * obj.sampleRateHz);
                beamPath(end+1:end+numsamples) = beamPath(1:numsamples);
            end
            
            beamPath(end) = 0;
        end
        
        function powers = powerDepthCorrectionFunc(obj,powers,z)            
            try
                switch obj.pzAdjust
                    case scanimage.types.BeamAdjustTypes.None
                        powers = powers.*ones(numel(z),1);
                        
                    case scanimage.types.BeamAdjustTypes.Exponential
                        powers = powers .* exp( (z-obj.pzReferenceZ) ./ obj.Lz );
                        
                    case scanimage.types.BeamAdjustTypes.Function
                        if isscalar(powers)
                            powers = repmat(powers,size(z));
                        end
                        
                        powers = obj.pzFunction(powers,z,obj);
                        
                        assert(isequal(size(powers),size(z)),'Custom power function returned vector of incorrect length. Expected length: %d. Actual:%d',numel(z),numel(powers));
                        
                    case scanimage.types.BeamAdjustTypes.LUT
                        if isempty(obj.pzLUT)
                            powers = zeros(size(z));
                        elseif size(obj.pzLUT,1) == 1
                            powers = repmat(obj.pzLUT(1,2),size(z));
                        else
                            interpolationMethod = 'linear';
                            extrapolationMethod = 'nearest';
                            hInt = griddedInterpolant(obj.pzLUT(:,1),obj.pzLUT(:,2),interpolationMethod,extrapolationMethod);
                            powers = hInt(z);
                        end
                        
                    otherwise
                        error('Unknown value for pzAdjust: %s',char(obj.pzAdjust));
                end
                
                powers = obj.enforcePowerLimit(powers);
            
            catch ME
                most.ErrorHandler.logAndReportError(ME);
                powers = zeros(size(z));
            end
        end
        
        function powers = enforcePowerLimit(obj,powers)
            upperLimit = min(1,obj.hDevice.powerFractionLimit);
            powers = max(powers,0);
            powers = min(powers,upperLimit);
        end
    end
    
    methods
        function val = get.powerFractionLimit(obj)
            val = obj.hDevice.powerFractionLimit;
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
