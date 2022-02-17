function check_stability()
  global Kx Ky Kz Kphi Ktheta Kpsi Jphi Jtheta Jpsi m g Ar N
  global pv1 pv2 pv3 Local
  for n = 1:N
    for i = 1:length(pv1)
       for j = 1:length(pv2)
          for k = 1:length(pv3)
              Local(n,i,j,k).Lambda = eig(Local(n,i,j,k).A+Local(n,i,j,k).B*Local(n,i,j,k).K);
              %Local(n,i,j,k).Lambda;
              %rank(Local(n,i,j,k).A+Local(n,i,j,k).B*Local(n,i,j,k).K);
          end %% pv1
       end %% pv2
    end %% pv3
  end %% ²Än¥xUAV
end