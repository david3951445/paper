function LMI_find_K(rho,Q)
  global Kx Ky Kz Kphi Ktheta Kpsi Jphi Jtheta Jpsi m g Ar mar_stable
  global pv1 pv2 pv3 Local
  mar_stable=0;
  Q1 = Q(1:12,:);
  Q2 = Q(13:24,:);
    for i1 = 1:length(pv1)
       for j1 = 1:length(pv2)
          for k1 = 1:length(pv3)
             for i2 = 1:length(pv1)
                for j2 = 1:length(pv2)
                   for k2 = 1:length(pv3)
              setlmis([])  %% initialize LMI
              P11 = lmivar(1,[12,1]);  %% specify variable matrix
              W22 = lmivar(1,[12,1]);  %% specify variable matrix
              W33 = lmivar(1,[12,1]);  %% specify variable matrix
              Y1 = lmivar(2,[4 12]);   %% specify variable matrix
              Y2 = lmivar(2,[4 12]);   %% specify variable matrix

              lmiterm([-2 1 1 P11],1,1);                      %% LMI #2 W11>0
              lmiterm([-3 1 1 W22],1,1);                      %% LMI #3 W22>0
              lmiterm([-4 1 1 W33],1,1);                      %% LMI #4 W33>0

              lmiterm([1 1 1 P11],1,Ar,'s');                  %% LMI #1 (1,1)
              lmiterm([1 1 1 0],(1/(4*rho^2))*eye(12));       %% LMI #1 (1,1)
              lmiterm([1 2 1 0],Local(i1,j1,k1,i2,j2,k2).A1); %% LMI #1 (2,1)
              lmiterm([1 2 1 0],-Ar);                         %% LMI #1 (2,1)
              
              lmiterm([1 2 2 Y1],Local(i1,j1,k1,i2,j2,k2).B1,1,'s');  %% LMI #1 (2,2)
              lmiterm([1 2 2 W22],Local(i1,j1,k1,i2,j2,k2).A1,1,'s'); %% LMI #1 (2,2)
              lmiterm([1 2 2 0],(1/(4*rho^2))*eye(12));       %% LMI #1 (2,2)
              
              lmiterm([1 3 2 W22],1,1);                       %% LMI #1 (3,2)
              
              lmiterm([1 3 3 0],-inv(Q1));                    %% LMI #1 (3,3)
              
              lmiterm([1 4 1 0],Local(i1,j1,k1,i2,j2,k2).A2);         %% LMI #1 (4,1)
              lmiterm([1 4 1 0],Local(i1,j1,k1,i2,j2,k2).A1);         %% LMI #1 (4,1)
              
              lmiterm([1 4 3 Y1],-Local(i1,j1,k1,i2,j2,k2).B1,1);     %% LMI #1 (4,3)
              lmiterm([1 4 3 W22],Local(i1,j1,k1,i2,j2,k2).A2,1);     %% LMI #1 (4,3)
              lmiterm([1 4 3 W22],-Local(i1,j1,k1,i2,j2,k2).A1,1);    %% LMI #1 (4,3)
              
              lmiterm([1 4 4 Y2],Local(i1,j1,k1,i2,j2,k2).B2,1,'s');  %% LMI #1 (4,4)
              lmiterm([1 4 4 W33],Local(i1,j1,k1,i2,j2,k2).A2,1,'s'); %% LMI #1 (4,4)
              lmiterm([1 4 4 0],(1/(4*rho^2))*eye(12));       %% LMI #1 (4,4)
              
              lmiterm([1 5 4 W33],1,1);                       %% LMI #1 (5,4)
              
              lmiterm([1 5 5 0],-inv(Q2));                    %% LMI #1 (5,5)

              lmis = getlmis;
              [tmin,xopt] = feasp(lmis);
              Y1 = dec2mat(lmis,xopt,Y1);
              Y2 = dec2mat(lmis,xopt,Y2);
              W22 = dec2mat(lmis,xopt,W22);
              W33 = dec2mat(lmis,xopt,W33);
              Local(i1,j1,k1,i2,j2,k2).K1 = Y1*inv(W22);
              Local(i1,j1,k1,i2,j2,k2).K2 = Y2*inv(W33);
              
              if tmin>=0
                  mar_stable=mar_stable+1;
              else
              end
                   end
                 end
             end
          end
       end
    end
end