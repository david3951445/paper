function ag = Load(ag)
%load old data
if isfile([ag.PATH '.mat'])
    data = load(ag.PATH);
    if isfield(data, 'tr')
        ag.tr = data.tr;
    end
    if isfield(data, 'K')
        ag.K = data.K;
    end
    if isfield(data, 'KL')
        ag.KL = data.KL;
    end
end

end