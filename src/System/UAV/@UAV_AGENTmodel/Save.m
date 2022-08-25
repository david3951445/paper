function Save(uav, whichVar)
%Save a property into classname.mat
% whichVar  : which property be saved

switch whichVar
    case 'tr'
        tr = uav.tr;
    case 'qr'
        qr = uav.qr;
    case 'K'
        K = uav.K;
    case 'KL'
        KL = uav.KL;
    otherwise
        disp(['No such property in uav'])
end

if isfile([uav.PATH '.mat'])
    save(uav.PATH, whichVar, '-append');
else
    save(uav.PATH, whichVar);
end
    
end