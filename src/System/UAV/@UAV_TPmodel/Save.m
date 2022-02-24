function Save(uav, whichVar)
    switch whichVar
        case 'AB'
            AB = uav.AB;
        otherwise
            Save@UAV(uav, whichVar); % The properties in superclass UAV()
            return
    end
    
    if isfile([uav.PATH '.mat'])
        save(uav.PATH, whichVar, '-append')
    else
        save(uav.PATH, whichVar)
    end           
end