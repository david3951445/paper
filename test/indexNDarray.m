clc; clear
len = 5*4*3*2;
S = zeros(5, 4, 3, 2);
for i = 1 : len
    S(i) = i;
end

len2 = 5*4;
A = cell(1, len2);
for i = 1 : len2
    A{i} = zeros(3, 2);
    for j = 1 : 3*2
        A{i}(j) = S(i + len2*(j-1));
    end
end
A{1}