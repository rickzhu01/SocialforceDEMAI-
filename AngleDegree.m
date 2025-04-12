function sigma =AngleDegree(OA,OB)
%ANGLE  sigma in degree  2 dimension vector 
%   Detailed explanation goes here

% check size function    
  arguments
        OA (1,2) double
        OB (1,2) double
   end
    
    if (isvector(OA) || isvector(OB))
          sigma = acos(dot(OA,OB) /  (len(OA(1,1), OA(1,2) )*len(OB(1,1), OB(1,2)))); % dot 
          sigma =sigma/pi*180;  % degree 
    else
          error(eidType,msgType)
    end 
    
    
end

