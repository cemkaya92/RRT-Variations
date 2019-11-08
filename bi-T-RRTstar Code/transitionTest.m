% This File is created by U. Cem Kaya - Spring 2019
%% TRANSITION TEST FOR MOVING TO HIGHER COST REGION
function [test,T] = transitionTest(cost_parent, cost_child, T)
    T_rate = T.rate;
    costRange = T.range;
    Temp = T.temp;
    if cost_child > cost_parent % moving uphill
        
        if exp(-(cost_child - cost_parent)/Temp) > 0.5 % depending on this probability, choose to move uphill
            test = 1;
            Temp = Temp/(2^((cost_child - cost_parent)/costRange));
        else
            test = 0;
            Temp = Temp*2^(T_rate);
        end
        T.temp = Temp;
        
    else
        test = 1;
    end

end