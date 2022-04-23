function Save(rb, whichVar)
%Save a property into classname.mat
% whichVar  : which property be saved

switch whichVar
    case 'qr'
        qr = rb.qr;
    otherwise
        disp(['No such property in rb'])
end

if isfile([rb.PATH '.mat'])
    save(rb.PATH, whichVar, '-append');
else
    save(rb.PATH, whichVar);
end
    
end