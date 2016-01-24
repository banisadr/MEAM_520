function [output] = team125_lspb_trajectory( t, t1,t2,thetas1,thetas2,v1,v2 )

tb = (t2-t1)/4;

 mat = [0       0           1       t1      t1^2        0       0        0;
        0       0           0       1       2*t1        0       0        0;
        -1      -(t1+tb)    1       t1+tb   (t1+tb)^2   0       0        0;
        0       -1          1       0       2*(t1+tb)   0       0        0;
        -1      -(t2-tb)    0       0       0           1       t2-tb   (t2-tb)^2;
        0       -1          0       0       0           0       1        2*(t2-tb);
        0       0           0       0       0           1       t2       t2^2;
        0       0           0       0       0           0       1        2*t2];
       
output = zeros(6,1);

for i = 1:6
    conditions = [thetas1(i), v1(i), 0, 0, 0, 0,thetas2(i),v2(i)]';
       
    coeff = mat\conditions;
    
    a0 = coeff(1);
    a1 = coeff(2);
    b0 = coeff(3);
    b1 = coeff(4);
    b2 = coeff(5);
    c0 = coeff(6);
    c1 = coeff(7);
    c2 = coeff(8);
    
    if t <= t1+tb
        output(i) = b0 + b1*t + b2*t^2;
        
    elseif t > t1+tb && t <= t2-tb
        output(i) = a0 + a1*t;
    else
        output(i) = c0 + c1*t + c2*t^2;
    end
            
    
end
        
end