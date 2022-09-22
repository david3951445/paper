function rb = Load(rb)
%load old data
rb = Load@Agent(rb);

if isfile([rb.PATH '.mat'])
    data = load(rb.PATH);
    
    if isfield(data, 'qr')
        rb.qr = data.qr;
    end
    if isfield(data, 'CoM')
        rb.CoM = data.CoM;
    end
end

end