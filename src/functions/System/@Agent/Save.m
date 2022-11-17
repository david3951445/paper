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
    case 'sys_a'
        sys_a = ag.sys_a;
    case 'sys_s'
        sys_s = ag.sys_s;
    case 'sys_aug'
        sys_aug = ag.sys_aug;
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