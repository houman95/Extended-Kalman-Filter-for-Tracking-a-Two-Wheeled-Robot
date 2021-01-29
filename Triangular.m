function val = Triangular(mx)
% val = Triangular(mx)
% 
% Implements a triangular distribution, centered, between -mx and mx.
%

% sample from uniform distribution in [0,1]
u = rand;

% Solve for sample of triangular distribution.
if u<=0.5
    val = mx*(-1+sqrt(2*u));
else
    val = mx*(1-sqrt(2-2*u));
end;
