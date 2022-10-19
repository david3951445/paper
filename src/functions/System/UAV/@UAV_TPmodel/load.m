function uav = load(uav)
    if isfile([uav.PATH '.mat'])
        data = load(uav.PATH);
        
        if isfield(data, 'AB')
            uav.AB = data.AB;
        end
    end

    uav = load@UAV(uav);
end