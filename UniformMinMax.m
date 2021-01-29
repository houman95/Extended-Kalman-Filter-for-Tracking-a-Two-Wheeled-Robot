function val = UniformMinMax(mn,mx)
% val = UniformMinMax(mn,mx)
% 
% Implements a uniform distribution between mn and mx.
%


val = mn + (mx-mn)*rand;