function Save(rb, whichVar)
%Save a property into classname.mat
% whichVar  : which property be saved

switch whichVar
    case 'tr'
        tr = rb.tr;
    case 'qr'
        qr = rb.qr;
    case 'CoM'
        CoM = rb.CoM;
    case 'K'
        K = rb.K;
    case 'KL'
        KL = rb.KL;
    otherwise
        disp(['No such property in rb'])
end

if isfile([rb.PATH '.mat'])
    save(rb.PATH, whichVar, '-append');
else
    save(rb.PATH, whichVar);
end
    
end