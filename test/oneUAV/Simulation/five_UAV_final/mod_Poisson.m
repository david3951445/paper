function [Poisson1 Poisson2 Poisson3 Poisson4 Poisson5]=mod_Poisson(L,Counting1,Counting2,Counting3,Counting4,Counting5)

for i=1:L-1
  if Counting1(i)==Counting1(i+1)
    Poisson1(i)=0;
  else
    Poisson1(i)=1;
  end
  if Counting2(i)==Counting2(i+1)
    Poisson2(i)=0;
  else
    Poisson2(i)=1;
  end
  if Counting3(i)==Counting3(i+1)
    Poisson3(i)=0;
  else
    Poisson3(i)=1;
  end
  if Counting4(i)==Counting4(i+1)
    Poisson4(i)=0;
  else
    Poisson4(i)=1;
  end
  if Counting5(i)==Counting5(i+1)
    Poisson5(i)=0;
  else
    Poisson5(i)=1;
  end


end