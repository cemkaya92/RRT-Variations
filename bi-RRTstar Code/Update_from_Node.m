% This file is created by U. Cem Kaya - 2018
function Path = Update_from_Node(Path,startingNodeID,costChange,timeChange,distanceChange)


    for id = startingNodeID:length(Path)
        
        Path(id).CumulativeCost = Path(id).CumulativeCost + costChange;
        Path(id).time = Path(id).time + timeChange;
        Path(id).totalDistance = Path(id).totalDistance + distanceChange;
        
    end



end