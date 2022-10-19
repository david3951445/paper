function Save(rb, whichVar)
%Save a property into classname.mat
% whichVar  : which property be saved
if Save@Agent(rb, whichVar)
    return
end

switch whichVar
    case 'r_lr'
        r_lr = rb.r_lr;
    case 'INTERP_DENSITY'
        INTERP_DENSITY = rb.INTERP_DENSITY;
    case 'rbtree'
        rbtree = rb.rbtree;
    case 'qr'
        qr = rb.qr;
    case 'CoM'
        CoM = rb.CoM;
    otherwise
        disp(['No such property in rb'])
end

if isfile([rb.PATH '.mat'])
    save(rb.PATH, whichVar, '-append');
else
    save(rb.PATH, whichVar);
end
    
end