function [output] = team125_cubic_trajectory( t, t1,t2,thetas1,thetas2,v1,v2 )

 mat =     [1    t1  t1^2    t1^3;
           0    1   2*t1    3*t1^2;
           1    t2  t2^2    t2^3;
           0    1   2*t2    3*t2^2];
       
output = zeros(6,1);

for i = 1:6
    conditions = [thetas1(i), v1(i),thetas2(i),v2(i)]';
       
    coeff = mat\conditions;
    
    a0 = coeff(1);
    a1 = coeff(2);
    a2 = coeff(3);
    a3 = coeff(4);
    
    output(i) = a0 + a1*t + a2*t^2 + a3*t^3;
    
end
        
end

