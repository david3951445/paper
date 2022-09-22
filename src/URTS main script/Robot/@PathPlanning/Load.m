function rb = Load(rb)
%load old data

if isfile([rb.PATH '.mat'])
    data = load(rb.PATH);

    % if isfield(data, 'qr')
    %     rb.qr = data.qr;
    % end

end

end