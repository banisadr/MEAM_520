function [output] = team125_linear_trajectory( t, t1,t2,thetas1,thetas2)

 mat =     [1 t1;
            1 t2];
       
output = zeros(6,1);

for i = 1:6
    conditions = [thetas1(i),thetas2(i)]';
       
    coeff = mat\conditions;
    
    a0 = coeff(1);
    a1 = coeff(2);
    
    output(i) = a0 + a1*t;
    
end
        
end
