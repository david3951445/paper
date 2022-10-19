function uav = Load(uav)
%load old data
uav = Load@Agent(uav);

if isfile([uav.PATH '.mat'])
    data = load(uav.PATH);

    if isfield(data, 'qr')
        uav.qr = data.qr;
    end
end

end