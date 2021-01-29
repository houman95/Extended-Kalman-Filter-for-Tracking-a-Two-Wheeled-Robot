function run(designPart)
% run(designPart)
%
% Main function for Extended Kalman Filter programming exercise.
% It is used to simulate the truth model, call the estimator and show the 
% results.
%
% The function is used to obtain the results for both estimator design 
% parts of the programming exercise:
%   designPart==1  -> Part 1
%   designPart==2  -> Part 2
%
%

% clear command window, close figures
clc;
close all;


%% Checks
% Check input argument.  designPart has to be either 1 or 2 corresponding
% to the two problem parts.
if nargin<1 || ((designPart~=1) && (designPart~=2))
    error('Wrong input argument.  You have to call run(1) or run(2) corresponding to the estimator design part considered.');
    return;
else
    disp(['Executing ''run'' for estimator design part #',num2str(designPart),'.']);
end;




%% Setup
% Define the physical constants used in the simulation and accessible to 
% the estimator.
knownConst = KnownConstants();

% Define the simulation constants that are used in simulation, but not 
% accessible to the estimator.  
unknownConst = UnknownConstants();

% Set the random number generator state.
% Uncomment to make results reproducable. This setting was used to generate
% the plot in the problem description.
rand('seed',1);
randn('seed',1);


%% Initialize Variables

%%%
% The following are invisible to the estimator
%%%

% The wheel base is not known exactly, it is modeled as random variable.
% The actual wheel base is obtained by drawing from a random number
% generator.
wheelBase = knownConst.NominalWheelBase * (1 + Triangular(knownConst.WheelBaseRelativeError));

% The wheel radius is not known exactly, it is modeled as random variable.
% The actual wheel radius is obtained by drawing from a random number
% generator.
wheelRadius = knownConst.NominalWheelRadius * (1 + Uniform(knownConst.WheelRadiusRelativeError));


% Time steps in simulation
N = floor(unknownConst.simulationTime/unknownConst.sampleContinuous);

% The robot position and orientation
loc = zeros(N+1,3);

% The starting position and orientation
loc(1,1) =  Uniform(knownConst.TranslationStartBound);
loc(1,2) =  Uniform(knownConst.TranslationStartBound);
loc(1,3) =  Uniform(knownConst.RotationStartBound);


%%%
% The following are visible to the estimator
%%%

% The commanded wheel speeds.  First is right, second is left.
wheelSpeeds = zeros(N,2);

% The compass sensor.  Set to "inf" when no reading is made.
compassSensor = zeros(N,1);

% The position sensor.  Set to "inf" when no reading is made.
posSensor = zeros(N,2);

% Time
tm = unknownConst.simulationTime * (1:N)'/N;



%% Simulation: Generate data
% Simulation of the system to obtain inputs, measurements and the true
% states.  Inputs and measurements will be visible to the estimator.
%
% In this simulation, the inputs are piecewise constant functions of time, 
% that is for a sequence of random length, the input is constant.

% The inputs for the first segment
[inputRight, inputLeft, segTime] = CalculateInputs(unknownConst,knownConst, wheelRadius);

% Measurements are acquired at randomized time intervals whose length is
% between the minimum and maximum length specified in UNKNOWNCONST.

% The next compass reading: if the simulation time exceeds COMPASSTIME, a
% new compass measurement is acquired.
compassTime = UniformMinMax(unknownConst.sampleCompassMin,unknownConst.sampleCompassMax);

% The next position reading: if the simulation time exceeds POSTIME, a
% new positio   n measurement (either x or y) is acquired.
posTime =  UniformMinMax(unknownConst.samplePosMin,unknownConst.samplePosMax);

for n = 1:N
    % Store control input (commanded wheel angular velocity, known to estimator)
    wheelSpeeds(n,1) = inputRight;
    wheelSpeeds(n,2) = inputLeft;
    
    % The actual translational and rotational velocities
    if designPart==1
        % Design Part 1: Commands followed instantaneously.
        inputRightAct = inputRight;
        inputLeftAct = inputLeft;
    else
        % Design Part 2: inputs corrupted by noise.
        inputRightAct = inputRight*(1+Triangular(knownConst.InputNoise));
        inputLeftAct = inputLeft*(1+Triangular(knownConst.InputNoise));
    end;
    speedTranslation = 0.5*wheelRadius*(inputRightAct + inputLeftAct);
    speedRotation = 0.5*wheelRadius*(inputRightAct - inputLeftAct)/wheelBase;
    
    % Simple fixed time step integration.  A little bit more complicated
    % than simple Euler integration, retains second order temporal terms.
    loc(n+1,1) = loc(n,1) + unknownConst.sampleContinuous*cos(loc(n,3))*speedTranslation - ...
            0.5*unknownConst.sampleContinuous^2*sin(loc(n,3))*speedRotation*speedTranslation;
    loc(n+1,2) = loc(n,2) + unknownConst.sampleContinuous*sin(loc(n,3))*speedTranslation + ...
            0.5*unknownConst.sampleContinuous^2*cos(loc(n,3))*speedRotation*speedTranslation;
    loc(n+1,3) = loc(n,3) + unknownConst.sampleContinuous*speedRotation;
    
    % Switch to new segment if we are at the end of the old one
    if (tm(n) > segTime);
        [inputRight, inputLeft, segTime] = CalculateInputs(unknownConst,knownConst, wheelRadius);
        segTime = segTime + tm(n);
    end    
    
    % The sensor reading.  The default is no measurement
    compassSensor(n) = inf;
    posSensor(n,1) = inf;
    posSensor(n,2) = inf;    
    
    % See if a compass reading is made
    if (tm(n) > compassTime)
        compassTime = tm(n) + UniformMinMax(unknownConst.sampleCompassMin,unknownConst.sampleCompassMax);
        compassSensor(n) = loc(n+1,3) + Triangular(knownConst.CompassNoise);
    end
    
    % See if a position reading is made.  Either X or Y, but not both.    
    if (tm(n) > posTime)
        posTime = tm(n) + UniformMinMax(unknownConst.samplePosMin,unknownConst.samplePosMax);
        if (rand > 0.5)
            posSensor(n,1) = loc(n+1,1) + Normal(0,knownConst.PosNoise);
        else
            posSensor(n,2) = loc(n+1,2) + Normal(0,knownConst.PosNoise);
        end
    end 
    
end



%% Estimator
% Run the estimator.

% Initialize the estimator.  
estState = [];
posEst = zeros(N+1,2);
oriEst = zeros(N+1,1);
posVar = zeros(N+1,2);
oriVar = zeros(N+1,1);
baseEst = zeros(N+1,1);
baseVar = zeros(N+1,1);
radiusEst = zeros(N+1,1);
radiusVar = zeros(N+1,1);
[posEst(1,:),oriEst(1,:),posVar(1,:),oriVar(1,:),baseEst(1,:),baseVar(1,:),radiusEst(1,:),radiusVar(1,:),estState] = ...
    Estimator(estState,zeros(1,2),zeros(1,3),0,knownConst,designPart);

% Call the estimator for each time step.
for n = 1:N
    [posEst(n+1,:),oriEst(n+1,:),posVar(n+1,:),oriVar(n+1,:),baseEst(n+1,:),baseVar(n+1,:),radiusEst(n+1,:),radiusVar(n+1,:),estState] = ...
        Estimator(estState,wheelSpeeds(n,:),[posSensor(n,:),compassSensor(n)],tm(n),knownConst,designPart);
end



%% The results
% Plots of the results.

% Calculate the total tracking error.  Basically the 2 norm of the distance
% error.
trackError = [loc(:,1) - posEst(:,1);loc(:,2) - posEst(:,2)];
trackErrorNorm = sqrt(trackError'*trackError/N);

%%%%%
% 2D-tracking plot
%%%%%

% Plot the actual robot position, y vs x, and the estimated robot position
% and orientation as arrows.
figure(1)
plot(loc(:,1),loc(:,2),'b.', posEst(:,1),posEst(:,2),'g.', ...
    loc(1,1),loc(1,2),'r*',loc(end,1),loc(end,2),'ro',posEst(1,1),posEst(1,2),'m*',posEst(end,1),posEst(end,2),'mo');
hold on;
quiver(loc(:,1), loc(:,2), cos(loc(:,3)), sin(loc(:,3)),0.08, 'b');
quiver(posEst(:,1), posEst(:,2), cos(oriEst), sin(oriEst),0.08, 'g');
%
grid;
legend('true','estimate','start true','end true','start est.','end est.');
xlabel('x position');
ylabel('y position');
title(['position tracking error: ',num2str(trackErrorNorm,6),' m']);


%%%%%
% estimation error (incl. standard deviation)
%%%%%

% plot estimation error together with +/- 1 standard deviation
figure(2);
tm_ = [0;tm];
%
subplot(5,1,1);
plot(tm_,loc(:,1)-posEst(:,1),tm_,sqrt(posVar(:,1)),'r',tm_,-sqrt(posVar(:,1)),'r');
grid;
ylabel('position x (m)');
title('Estimation error with +/- standard deviation');
%
subplot(5,1,2);
plot(tm_,loc(:,2)-posEst(:,2),tm_,sqrt(posVar(:,2)),'r',tm_,-sqrt(posVar(:,2)),'r');
grid;
ylabel('position y (m)');
%
subplot(5,1,3);
plot(tm_,loc(:,3)-oriEst,tm_,sqrt(oriVar),'r',tm_,-sqrt(oriVar),'r');
grid;
ylabel('orientation r (rad)');
%
subplot(5,1,4);
plot(tm_,wheelBase-baseEst,tm_,sqrt(baseVar),'r',tm_,-sqrt(baseVar),'r');   
grid;
ylabel('wheel base B (m)');
%
subplot(5,1,5);
plot(tm_,wheelRadius-radiusEst,tm_,sqrt(radiusVar),'r',tm_,-sqrt(radiusVar),'r');
grid;
xlabel('time (s)');
ylabel('wheel radius W (m)');

return;

    