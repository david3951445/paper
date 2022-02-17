function [K1 A1 B1 K2 A2 B2] = fuzzy_interpolation(x)
  global Kx Ky Kz Kphi Ktheta Kpsi Jphi Jtheta Jpsi m g Ar num
  global pv1 pv2 pv3 Local
  Total = 0;
  K1 = zeros(4,12);
  A1 = zeros(12);
  B1 = zeros(12,4);
  K2 = zeros(4,12);
  A2 = zeros(12); 
  B2 = zeros(12,4);
  for n = 1:num
     for i1 = 1:length(pv1)
        for j1 = 1:length(pv2)
           for k1 = 1:length(pv3)
              for i2 = 1:length(pv1)
                 for j2 = 1:length(pv2)
                    for k2 = 1:length(pv3)
                       M1 = 0; M2 = 0; M3 = 0;
                       M4 = 0; M5 = 0; M6 = 0;
              %% pv1 i1
                 if i1 == 1
                     M1 = trapmf(x(7),[-inf -inf pv1(1) pv1(2)]);
                 elseif i1 == 2
                     M1 = trimf(x(7),[pv1(1) pv1(2) pv1(3)]);
                 else
                     M1 = trapmf(x(7),[pv1(2) pv1(3) inf inf]);
                 end

             %% pv2 j1
                 if j1 == 1
                     M2 = trapmf(x(9),[-inf -inf pv2(1) pv2(2)]);
                 elseif j1 == 2
                     M2 = trimf(x(9),[pv2(1) pv2(2) pv2(3)]);
                 else 
                     M2 = trapmf(x(9),[pv2(2) pv2(3) inf inf]);
                 end

             %% pv3 k1
                 if k1 == 1
                     M3 = trapmf(x(11),[-inf -inf pv3(1) pv3(2)]);
                 elseif k1 == 2
                     M3 = trimf(x(11),[pv3(1) pv3(2) pv3(3)]);
                 else 
                     M3 = trapmf(x(11),[pv3(2) pv3(3) inf inf]);
                 end
                 
              %% pv1 i2
                 if i2 == 1
                     M4 = trapmf(x(19),[-inf -inf pv1(1) pv1(2)]);
                 elseif i2 == 2
                     M4 = trimf(x(19),[pv1(1) pv1(2) pv1(3)]);
                 else
                     M4 = trapmf(x(19),[pv1(2) pv1(3) inf inf]);
                 end

             %% pv2 j2
                 if j2 == 1
                     M5 = trapmf(x(21),[-inf -inf pv2(1) pv2(2)]);
                 elseif j2 == 2
                     M5 = trimf(x(21),[pv2(1) pv2(2) pv2(3)]);
                 else 
                     M5 = trapmf(x(21),[pv2(2) pv2(3) inf inf]);
                 end

             %% pv3 k2
                 if k2 == 1
                     M6 = trapmf(x(23),[-inf -inf pv3(1) pv3(2)]);
                 elseif k2 == 2
                     M6 = trimf(x(23),[pv3(1) pv3(2) pv3(3)]);
                 else 
                     M6 = trapmf(x(23),[pv3(2) pv3(3) inf inf]);
                 end

        %% ²£¥Í mu ©M summation mu
                mu(i1,j1,k1,i2,j2,k2) = M1*M2*M3*M4*M5*M6;
                Total = Total+mu(i1,j1,k1,i2,j2,k2);
                    end
                 end
              end
           end
        end
     end
  end
Su = 0;

   for i1 = 1:length(pv1)
      for j1 = 1:length(pv2)
         for k1 = 1:length(pv3)
            for i2 = 1:length(pv1)
               for j2 = 1:length(pv2)
                  for k2 = 1:length(pv3)
                %Su = Su + (mu(i,j,k)/Total);
                 K1 = K1+ (mu(i1,j1,k1,i2,j2,k2)/Total)*Local(i1,j1,k1,i2,j2,k2).K1;
                 K2 = K2+ (mu(i1,j1,k1,i2,j2,k2)/Total)*Local(i1,j1,k1,i2,j2,k2).K2;
                 A1 = A1+ (mu(i1,j1,k1,i2,j2,k2)/Total)*Local(i1,j1,k1,i2,j2,k2).A1;
                 A2 = A2+ (mu(i1,j1,k1,i2,j2,k2)/Total)*Local(i1,j1,k1,i2,j2,k2).A2;
                 B1 = B1+ (mu(i1,j1,k1,i2,j2,k2)/Total)*Local(i1,j1,k1,i2,j2,k2).B1;
                 B2 = B2+ (mu(i1,j1,k1,i2,j2,k2)/Total)*Local(i1,j1,k1,i2,j2,k2).B2;
                  end
               end
            end
         end
      end
   end





