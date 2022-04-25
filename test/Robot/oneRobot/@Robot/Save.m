function Save(rb, whichVar)
%Save a property into classname.mat
% whichVar  : which property be saved

switch whichVar
    case 'qr'
        qr = rb.qr;
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