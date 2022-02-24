function uav = load(uav)
%load old data

if isfile([uav.PATH '.mat'])
    data = load(uav.PATH);

    if isfield(data, 'A')
        uav.A = data.A;
    end
    if isfield(data, 'B')
        uav.B = data.B;
    end
    if isfield(data, 'AB')
        uav.AB = data.AB;
    end
    if isfield(data, 'K')
        uav.K = data.K;
    end
    if isfield(data, 'tr')
        uav.tr = data.tr;
    end
end

end