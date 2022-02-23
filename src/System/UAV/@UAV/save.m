function Save(obj, filename, whichVar)
%Save a property into UAV_FZmodel.mat
% whichVar  : which property be saved
% filename  : name of saved file

PATH = [obj.DATA_FOLDER_PATH filename];

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
        disp(['No such property in UAV'])
end

if isfile([PATH '.mat'])
    save(PATH, whichVar, '-append');
else
    save(PATH, whichVar);
end
    
end