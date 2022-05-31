function y = CC(validator, r) 
%collision checking

if isempty(r)
    y = false;
    return;
end

for i = 1 : size(r, 1) - 1
    state1 = [r(i,:) 0];
    state2 = [r(i+1,:) 0];
    [isValid, lastValid] = isMotionValid(validator, state1, state2);
    if ~isValid
        y = true;
        disp('CC')
        return
    end
end
y = 0;

% for i = 1 : size(r, 1s)
%     y = checkOccupancy(map, r(i, :));
%     if y
%         disp('CC')
%         return
%     end
% end
end

