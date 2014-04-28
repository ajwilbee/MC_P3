function [ y, time ] = Simulate(A, B, C, D, U, initialCondition,count)
%
%  
% find the discrete time equivelant( zero order hold ...ect) then compute

   
    time = 1:1:count;
    y = cell(count,1);
%     x = [-1.94 -1.65 -.78 .45 -.72 .32 -.87]';
    x = initialCondition;
    for z = 1:count
        Xdot = A*x+B*U;
        y{z} = C*x+D*U;
        x = Xdot;
    end



end

