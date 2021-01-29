function [inputRight, inputLeft, segTime] = CalculateInputs(unknownConst,knownConst, wheelRadius)
% [inputRight, inputLeft, segTime] = CalculateInputs(unknownConst,knownConst, wheelRadius)
% 
% Calculate the input wheel speeds, in rad/s, and the duration of the 
% segment in seconds.  This is only called during the simulation; not from
% the estimator.
%
% In this implementation, the inputs are piecewise constant functions of 
% time, that is for a segment of random length (segTime), the inputs
% (inputRight, inputLeft) are constant.
%
% Input:
%   unknownConst    constants not known to estimator (from UnknownConstants.m)
%   knownConst      constants known to estimator (from KnownConstants.m)
%   wheelRadius     wheelRadius for this simulation (random, constant parameter)
%
% Output:
%   inputRight      right wheel angular velocity command (in rad/s)
%   inputLeft       left wheel angular velocity command (in rad/s)
%   segTime         duration of segment (in s) for that the inputs are
%                   constant
%

%
% --
% Revision history


    
% The length of the segment, in seconds.
% Draw from uniform distribution.
segTime = UniformMinMax(unknownConst.minSegTime,unknownConst.maxSegTime);

% The desired translational and rotational speed.  
% Draw from uniform distribution. Note that the rotational speed is +-, 
% while the translational speed is only +.
speedTranslation = UniformMinMax(0,unknownConst.MaxSpeedTranslation);
speedRotation = Uniform(unknownConst.MaxSpeedRotation);

% Convert to commanded wheel speeds using the nominal wheel base and true
% wheel radii.
inputRight = (speedTranslation + knownConst.NominalWheelBase*speedRotation)/wheelRadius;
inputLeft =  (speedTranslation - knownConst.NominalWheelBase*speedRotation)/wheelRadius;
