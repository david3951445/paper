function Save(pp, whichVar)
%Save a property into classname.mat
% whichVar  : which property be saved

switch whichVar
    % case 'qr'
    %     qr = rb.qr;
    otherwise
        disp(['No such property in rb'])
end

if isfile([pp.PATH '.mat'])
    save(pp.PATH, whichVar, '-append');
else
    save(pp.PATH, whichVar);
end
    
end