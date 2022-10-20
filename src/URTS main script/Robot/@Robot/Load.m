function rb = Load(rb)
%load old data
rb = Load@Agent(rb);

if isfile([rb.PATH '.mat'])
    data = load(rb.PATH);
    
    if isfield(data, 'r_lr')
        rb.r_lr = data.r_lr;
    end
    % if isfield(data, 'INTERP_DENSITY')
    %     INTERP_DENSITY = rb.INTERP_DENSITY;
    % end
    % if isfield(data, 'rbtree')
    %     rb.rbtree = data.rbtree;
    % end
    if isfield(data, 'qr')
        rb.qr = data.qr;
    end
    if isfield(data, 'CoM')
        rb.CoM = data.CoM;
    end
end

end