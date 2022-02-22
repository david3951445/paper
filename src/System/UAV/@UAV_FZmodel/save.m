function save(obj, whichVar)
%Save a property into UAV_FZmodel.mat
% whichVar  : which property be saved
whichVar
switch whichVar
    case 'A'
        A = obj.A;
    case 'B'
        B = obj.B;
    case 'K'
        K = obj.K;
    case 'tr'
        tr = obj.tr;
    otherwise
        disp(['No such property in ' mfilename])
end
obj.DATA_PATH
if isfile([obj.DATA_PATH '.mat'])
    save(obj.DATA_PATH, whichVar, '-append');
else
    save(obj.DATA_PATH, whichVar);
end
    
end