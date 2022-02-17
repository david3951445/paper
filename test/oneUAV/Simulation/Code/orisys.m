function output=orisys(x,xhat,i,fa,fs)
global Abar Bbar Bfbar Cbar Dbar h x1 x3 x5 x6 sys con obs h T
% The completness has been ensured.
Total = 0;output = zeros(14,1);
for i = 1:length(x1)
   for j = 1:length(x3)
      for k = 1:length(x5)
          for l = 1:length(x6)
          M1 = 0; M2 = 0; M3 = 0; M4 = 0;
          if i == 1
          M1 = trapmf(x(9),[-inf -inf x1(1) x1(2)]);
          else if i == 2
               M1 = trimf(x(9),[x1(1) x1(2) x1(3)]);
           else if i == 3
               M1 = trimf(x(9),[x1(2) x1(3) x1(4)]);     
               
               else
               M1 = trapmf(x(9),[x1(3) x1(4) inf inf]);
               end
               end
          
          end
         
       if j == 1
           M2 = trapmf(x(11),[-inf -inf x3(1) x3(2)]);
       else if j == 2
               M2 = trimf(x(11),[x3(1) x3(2) x3(3)]);
          
                else
                       M2 = trapmf(x(11),[x3(2) x3(3) inf inf]);
           end
       end
       
       if k == 1
           M3 = trapmf(x(13),[-inf -inf x5(1) x5(2)]);
       else if k == 2
               M3 = trimf(x(13),[x5(1) x5(2) x5(3)]);
           else if k == 3
                   M3 = trimf(x(13),[x5(2) x5(3) x5(4)]);
             
                   else
                       M3 = trapmf(x(13),[x5(3) x5(4) inf inf]);
                   end
           end
           end
      
       if l == 1
           M4 = trapmf(x(14),[-inf -inf x6(1) x6(2)]);
       else if l == 2
               M4 = trimf(x(14),[x6(1) x6(2) x6(3)]);
           else if l == 3
                   M4 = trimf(x(14),[x6(2) x6(3) x6(4)]);
             
                    else
                          M4 = trapmf(x(14),[x6(3) x6(4) inf inf]); 
               end
           end
                 
           
       end
             mu(i,j,k,l) = M1*M2*M3*M4;
             Total = Total+mu(i,j,k,l);
          end
      end
   end
end
Su = 0;

for i = 1:length(x1)
   for j = 1:length(x3)
      for k = 1:length(x5)
          for l = 1:length(x6)
              output = output + (mu(i,j,k,l)/Total)*(Bbar*con(i,j,k,l).K*x);
            
            
                    
              
              
              
              Su = Su + (mu(i,j,k,l)/Total);
          end
      end
   end
end
% externel
%  output=output+[ -100*x(1) 100*x(1)-100*x(2) 100*x(2)-100*x(3) 100*x(3)-100*x(4) 100*x(4)-100*x(5)   .....
%                x(9)    ...
%                x(10)/(x(6)*cos(x(8))).....    
%               x(11)/x(6) ....
%               (((x(10)^2)+(x(11)^2))/x(6))+x(1) ...
%               -((x(9)*x(10))/x(6))+((x(10)*x(11)*tan(x(8)))/x(6))+x(1) .....
%               -((x(9)*x(11))/x(6))-((x(10)^2)*tan(x(8))/x(6))+x(1)]';
%               output=output+[100*f 0 0 0 0 ...
%               0 0 0 ...
%                    2*h*i*x(6) ...
%                   -2*h*i*x(10)*x(7)/sqrt(x(10)*x(10)+x(11)*x(11)) ...
%                   -2*h*i*x(11)*x(8)/sqrt(x(10)*x(10)+x(11)*x(11))]';
% non externel
output=output+[ -100*x(1)...
    100*x(1)-100*x(2)...
    100*x(2)-100*x(3)...
    100*x(3)-100*x(4)....
    -100*x(5)   .....
    100*x(5)-100*x(6)...
    100*x(6)-100*x(7)...
    100*x(7)-100*x(8)....
               x(12)    ...
               x(13)/(x(9)*cos(x(11))).....    
              x(14)/x(9) ....
              (((x(13)^2)+(x(14)^2))/x(9)) ...
              -((x(12)*x(13))/x(9))+((x(13)*x(14)*tan(x(11)))/x(9)) .....
              -((x(12)*x(14))/x(9))-((x(13)^2)*tan(x(11))/x(9))]';
              output=output+[100*fa 0 0 0 ...
                  100*fs 0 0 0....
              0 0 0 2*fa 2*fa 2*fa]';
                  
             

end