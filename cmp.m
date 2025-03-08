function result  = cmp(x,y)
% comparision of two numbers
%   Detailed explanation goes here

if   (abs(x - y) < eps)
    result = 0;
     
elseif (x - y < 0)
         result = -1;
else
    
 result = 1;

end

