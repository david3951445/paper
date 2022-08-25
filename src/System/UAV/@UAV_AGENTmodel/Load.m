function uav = Load(uav)
%load old data

if isfile([uav.PATH '.mat'])
    data = load(uav.PATH);

    if isfield(data, 'tr')
        uav.tr = data.tr;
    end
    if isfield(data, 'qr')
        uav.qr = data.qr;
    end
    if isfield(data, 'K')
        uav.K = data.K;
    end
    if isfield(data, 'KL')
        uav.KL = data.KL;
    end
end

end