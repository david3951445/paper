function Save(uav, whichVar)
%Save a property into classname.mat
% whichVar  : which property be saved
if Save@Agent(uav, whichVar)
    return
end

switch whichVar
    case 'qr'
        qr = uav.qr;
    otherwise
        disp(['No such property in uav'])
end

if isfile([uav.PATH '.mat'])
    save(uav.PATH, whichVar, '-append');
else
    save(uav.PATH, whichVar);
end
    
end