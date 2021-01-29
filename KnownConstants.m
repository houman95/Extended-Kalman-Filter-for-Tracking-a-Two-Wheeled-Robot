function const = KnownConstants()
% const = KnownConstants()
% 
% Define the physical constants that are available to the estimator.
%



%% Robot kinematic constants

% The nominal right and left wheel radius (W_0), in meter.
const.NominalWheelRadius = 0.1;

% The variability in the wheel radius is captured by a uniform distribution, 
% with the following bound (\bar{\gamma}), as a fraction of the nominal wheel 
% radii.  Dimensionless. +-
const.WheelRadiusRelativeError = 0.1; % const.WheelRadiusRelativeError = \bar{gamma}

% The nominal wheel base (B_0), in meters.
const.NominalWheelBase = 0.5;

% The variability in the wheel base is captured by a triangular distribution, 
% with the following bound (\bar{\beta}), as a fraction of the nominal wheel 
% base.  Dimensionless. +-
const.WheelBaseRelativeError = 0.2; % const.WheelBaseRelativeError = \bar{\beta}


%% Noise properties

% The compass sensor noise (w_r), triangular distributed in 
% [-\bar{w},\bar{w}], units rad.
const.CompassNoise = 0.2; % const.CompassNoise = \bar{w}

% The position sensor noise (w_x, w_y), normally distributed with zero mean 
% and variance \sigma_p^2, units m^2.
const.PosNoise = 0.02; % const.PosNoise = \sigma_p^2

% Noise on wheel commands (\bar{v}); multiplicative noise, triangular 
% distribution in [-\bar{v},\bar{v}].
% This is only relevant for estimator design part 2.
const.InputNoise = 0.5; % const.InputNoise = \bar{v}

%% Starting point

% The robot nominally starts at the origin, uniformly distributed with the
% following bound (\bar{p}), in meters.
const.TranslationStartBound = 1.0; % const.TranslationStartBound = \bar{p}

% The nominal orientation is also 0, uniformly distributed with the
% following bound (\bar{r}), in rad.
const.RotationStartBound = -pi/2; % const.RotationStartBound = \bar{r}