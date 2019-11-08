% This file is created by U. Cem Kaya - 2018
%% FUNCTION TO CALCULATE HEADING
function theta = heading(position1,position2)

theta = atan2((position2.y-position1.y),(position2.x-position1.x));

end 