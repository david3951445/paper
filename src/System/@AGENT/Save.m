function is_saved = Save(ag, whichVar)
%Save a property into classname.mat
% whichVar  : which property be saved
is_saved = true;
switch whichVar
    case 'tr'
        tr = ag.tr;
    case 'K'
        K = ag.K;
    case 'KL'
        KL = ag.KL;
    otherwise
        is_saved = false;
        return;
end

if isfile([ag.PATH '.mat'])
    save(ag.PATH, whichVar, '-append');
else
    save(ag.PATH, whichVar);
end
    
end