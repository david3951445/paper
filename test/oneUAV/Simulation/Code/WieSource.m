%  T denote the total time, N denote the increment size
function dW=WieSource(dt,N) %  T denote the total time, N denote the increment size
dW = sqrt(dt)*randn(1,N); 