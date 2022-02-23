function uav = load(uav, filename)
%load old data

PATH = [uav.DATA_FOLDER_PATH filename];
if isfile([PATH '.mat'])
    data = load(PATH);

    if isfield(data, 'A')
        uav.A = data.A;
    end
    if isfield(data, 'B')
        uav.B = data.B;
    end
    if isfield(data, 'K')
        uav.K = data.K;
    end
    if isfield(data, 'tr')
        uav.tr = data.tr;
    end
end

end