function val = Normal(mx, vx)
% val = Normal(mx, vx)
% 
% Implements a normal distribution with mean mx and variance vx.
%
% 

if(vx < 0)
    error('Variance vx must be positive argument passed to ''Normal''')
end
val = mx + 2*sqrt(vx)*randn;