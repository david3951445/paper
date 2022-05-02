function rb = Load(rb)
%load old data

if isfile([rb.PATH '.mat'])
    data = load(rb.PATH);

    if isfield(data, 'tr')
        rb.tr = data.tr;
    end
    if isfield(data, 'qr')
        rb.qr = data.qr;
    end
    if isfield(data, 'CoM')
        rb.CoM = data.CoM;
    end
    if isfield(data, 'K')
        rb.K = data.K;
    end
    if isfield(data, 'KL')
        rb.KL = data.KL;
    end
end

end