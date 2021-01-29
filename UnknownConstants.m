function const = UnknownConstants()
% const = UnknownConstants()
% 
% Define the constants used in the simulation.  These constants are not 
% accessible to the estimator.
%



%% Speed

% The approximate maximum forward speed, in m/s.
const.MaxSpeedTranslation = 1.0;

% The approximate maximum rotational speed, in rad/s.
const.MaxSpeedRotation = 1.0;

% Minimum time for a segment, in seconds.
% Should be multiple of sampling time.
const.minSegTime = 0.5;

% Maximum time for a segment, in seconds.
% Should be multiple of sampling time.
const.maxSegTime = 2;



%% Times

% The duration of the simulation, in seconds.
const.simulationTime = 50;

% The sample time for the continuous dynamics, in seconds.
const.sampleContinuous = 0.1;

% The min sample time for the compass, in seconds.
const.sampleCompassMin = 1.0;

% The max sample time for the compass, in seconds.
const.sampleCompassMax = 3.0;

% The min sample time for the position sensors, in seconds.
const.samplePosMin = 2.0;

% The max sample time for the position sensors, in seconds.
const.samplePosMax = 4.0;

