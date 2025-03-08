function Distance= point2vector(p0,p1,p2)
%POINT2VECTOR 
   % p0-point  (x0,y0)   vecter from   p1(x1,y1)  to  p2 (x2,y2)  
%   Detailed explanation goes here
   arguments
        p0 (1,2) double
        p1 (1,2) double
        p2 (1,2) double
   end
    
    if (isvector(po) || isvector(p1)|| isvector(p2))
        
        Distance= abs( (p2(1,1)-p1(1,1))*(p1(1,2)-p0(1,2)) -(p1(1,1)-p0(1,1))*(p2(1,2)-p1(1,2)) )/sqrt((p2(1,1)-p1(1,1))^2 +(p2(1,2)-p1(1,2))^2 ) ;        
    
    end  
    
end

