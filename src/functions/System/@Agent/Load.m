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
    if isfield(data, 'sys_a')
        ag.sys_a = data.sys_a;
    end
    if isfield(data, 'sys_s')
        ag.sys_s = data.sys_s;
    end
    if isfield(data, 'sys_aug')
        ag.sys_aug = data.sys_aug;
    end
end