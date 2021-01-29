function [posEst,oriEst,posVar,oriVar,baseEst,baseVar,radiusEst,radiusVar,estState] = Estimator(estState,actuate,sense,tm,knownConst,designPart)
% [posEst,oriEst,posVar,oriVar,baseEst,baseVar,radiusEst,radiusVar,estState] =
% 	Estimator(estState,actuate,sense,tm,knownConst,designPart)
% 
% The estimator.
%
% The Estimator function shall be used for both estimator design parts; the
% input argument designPart is used to distinguish the tradiusEsto:
%   designPart==1  -> Part 1
%   designPart==2  -> Part 2
%
% The function radiusEstill be called in tradiusEsto different modes:
% If tm==0, the estimator is initialized; otherradiusEstise the estimator does an 
% iteration step (compute estimates for the time step k).
%
% Inputs:
%   estState        previous estimator state (time step k-1)
%                   May be defined by the user (for example as a struct).
%   actuate         control input u[k], [1x2]-vector
%                   actuate(1): u_R, right Wheel angular velocity
%                   actuate(2): u_L, left Wheel angular velocity
%   sense           sensor measurements z[k], [1x3]-vector,
%                   Single element = INF if no measurement from that sensor
%                   sense(1): z_x[k], x position measurement
%                   sense(2): z_y[k], y position measurement
%                   sense(3): z_r[k], orientation measurement
%   tm              time, continuous, scalar
%                   If tm==0 initialization, otherradiusEstise estimator
%                   iteration step.
%   knownConst      known constants (from knownConstants.m)
%   designPart      variable to distinguish the estimator design part 
%                       designPart==1  -> Part 1
%                       designPart==2  -> Part 2
%
% Outputs:
%   posEst          position estimate (time step k), [1x2]-vector
%                   posEst(1): x position estimate
%                   posEst(2): y position estimate
%   oriEst          orientation estimate (time step k), scalar
%   posVar          variance of position estimate (time step k), [1x2]-vector
%                   posVar(1): x position variance
%                   posVar(2): y position variance
%   oriVar          variance of orientation estimate (time step k), scalar
%   baseEst         estimate of Wheel base B, scalar
%   baseVar         variance of Wheel base estimate, scalar
%   radiusEst       estimate of Wheel radius radiusEst, scalar
%   radiusVar       variance of Wheel radius estimate, scalar
%   estState        current estimator state (time step k)
%                   radiusEstill be input to this function at the next call.
%
% 
% Class:
% Estimation
% Fall 2019
% Programming Exercise 1
%
% Writer: Hooman Asgari
% --
% Sharif Univ of Tech
%Instructor: Amin Rezaeizadeh
% aminre@sharif.ir
%
% --
% Revision history




%% Mode 1: Initialization
if (tm == 0)
    % Do the initialization of your estimator here!
    estState.time = tm;
    % Replace the folloradiusEsting:
    posEst = [0,0];
    oriEst = 0;
    posVar = [knownConst.TranslationStartBound^2,knownConst.TranslationStartBound^2]/3;
    oriVar = (knownConst.RotationStartBound^2)/3;
    baseEst = knownConst.NominalWheelBase;
    baseVar = knownConst.NominalWheelBase^2 * knownConst.WheelBaseRelativeError^2/6;
    radiusEst = knownConst.NominalWheelRadius;
    radiusVar = knownConst.NominalWheelRadius^2 * (knownConst.WheelRadiusRelativeError^2)/3;
    estState.state = [posEst,oriEst,baseEst,radiusEst]'
    estState.variance = diag([posVar,oriVar,baseVar,radiusVar])
        return;
end


%% Mode 2: Estimator iteration.
% If radiusEste get this far, tm is not equal to zero, and radiusEste are no longer
% initializing. Run the estimator.

% Implement your estimator here!
% Replace the folloradiusEsting:
T = tm - estState.time;

baseEst = estState.state(4);
radiusEst = estState.state(5);
% baseEst = knownConst.NominalWheelBase;
% radiusEst = knownConst.NominalWheelRadius;

st = mean(actuate)*radiusEst;
sr = radiusEst*(actuate(1) - actuate(2))/(2*baseEst);
prevXpost =  estState.state(1);
prevYpost =  estState.state(2);
prevRpost =  estState.state(3);
F = [...
        1, 0, -T*st*sin(prevRpost) - (1/2)*T^2*st*sr*cos(prevRpost), 0.5*T^2*st*(actuate(1) - actuate(2))/(2*baseEst^2)*sin(prevRpost), (T)*(mean(actuate))*cos(prevRpost) - (1/4)*T^2*(radiusEst/baseEst)*(actuate(1)^2 - actuate(2)^2)*sin(prevRpost);
        0, 1,  T*st*cos(prevRpost) - (1/2)*T^2*st*sr*sin(prevRpost),-0.5*T^2*st*(actuate(1) - actuate(2))/(2*baseEst^2)*cos(prevRpost), (T)*(mean(actuate))*sin(prevRpost) + (1/4)*T^2*(radiusEst/baseEst)*(actuate(1)^2 - actuate(2)^2)*cos(prevRpost);
        0, 0, 1, radiusEst*(actuate(2) - actuate(1))/(2*baseEst^2), (actuate(1) - actuate(2))/(2*baseEst);
        0, 0, 0, 1, 0;
        0, 0, 0, 0, 1];
    
L = [...
        T*radiusEst*actuate(1)/2*cos(prevRpost) - ((T^2)/(4*baseEst))*(radiusEst*actuate(1))^2*sin(prevRpost), -T*radiusEst*actuate(2)/2*cos(prevRpost) + ((T^2)/(4*baseEst))*(radiusEst*actuate(2))^2*sin(prevRpost);
        T*radiusEst*actuate(1)/2*sin(prevRpost) + ((T^2)/(4*baseEst))*(radiusEst*actuate(1))^2*cos(prevRpost), -T*radiusEst*actuate(1)/2*sin(prevRpost) - ((T^2)/(4*baseEst))*(radiusEst*actuate(1))^2*cos(prevRpost);
        T*radiusEst*actuate(1)/(2*baseEst), -T*radiusEst*actuate(2)/(2*baseEst)
        0, 0
        0, 0];

x_prior = prevXpost + T*st*cos(prevRpost) - 0.5*T^2*st*sr*sin(prevRpost);
y_prior = prevYpost + T*st*sin(prevRpost) + 0.5*T^2*st*sr*cos(prevRpost);
r_prior = prevRpost + T*sr;
B_prior = baseEst;
W_prior = radiusEst;

if(designPart==1)
    P_prior = F*estState.variance*F';
elseif(designPart==2) 
    P_prior = F*estState.variance*F' + L*diag([knownConst.InputNoise^2/6, knownConst.InputNoise^2/6])*L';
end

H = [...
        1, 0, 0, 0, 0;
        0, 1, 0, 0, 0;
        0, 0, 1, 0, 0];

M = [...
        1, 0, 0;
        0, 1, 0;
        0, 0, 1];
    
        hOfprior = [x_prior;y_prior;r_prior];
        z = sense;    
        hOfprior(find(isinf(sense))) = [];
        z(find(isinf(sense))) = [];        
        H(find(isinf(sense)),:) = [];
        M(find(isinf(sense)),:) = [];
estPrior = [x_prior;y_prior;r_prior;B_prior;W_prior];

if (isempty(H) == 0)
    K = P_prior*H'*pinv(H*P_prior*H' + M*diag([knownConst.PosNoise knownConst.PosNoise knownConst.CompassNoise^2/6])*M');
    estPost = estPrior + K*(z' - hOfprior);
    estState.variance = (eye(size(K*H)) - K*H)*P_prior;
else
    estPost = estPrior;
    estState.variance = P_prior;
end

posEst = estPost(1:2);
oriEst = estPost(3);
posVar = [estState.variance(1,1) estState.variance(2,2)];
oriVar =  estState.variance(3,3);
baseEst = estPost(4);
baseVar = estState.variance(4,4);
radiusEst = estPost(5);
radiusVar = estState.variance(5,5);
estState.state = estPost;
estState.time = tm;
estState.variance = (estState.variance + estState.variance')/2;
estPost;
end