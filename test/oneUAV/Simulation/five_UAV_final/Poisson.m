function Times = Poisson(lamda,Count)
    mu = rand(1,Count);
    Times = 0;
    for i = 1:Count
       Times(i+1) = Times(i) + (-log(1-mu(i)))/lamda;
    end
    Times = Times(2:length(Times));
end