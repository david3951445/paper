% % old
% function y = mf_A(o, x, ind)
% %Calculate mbfun of ind-th
% n = length(o.A.mf_discrete);

% y = 1;
% for i = 1 : n % multiply all "premise variable"
%     xS = o.A.mf_discrete{i}.x;
%     U = o.A.mf_discrete{i}.y;
%     k = ind(i);

%     m = length(xS);
%     if x < xS(1)
%         y = y*U(1, k);
%     elseif x > xS(n)
%         y = y*U(n, k);
%     else
%         for j = 2 : m
%             if x < xS(j)
%                 slope = (U(j, k) - U(j-1, k))/(xS(j) - xS(j-1));
%                 y = y*(U(j-1, k) + slope*(x - xS(j-1)));
%                 break;
%             end
%         end
%     end
        
%     % if x < xS(1)       
%     % elseif x < xS(i)
%     %     h1 = (x-xS(i-1))/(xS(i)-xS(i-1));
%     %     h2 = 1-h1;
%     %     %%% 分點 %%%
%     %     y = y*h1*U(i, ind(i)) + h2*U(i-1, ind(i));
%     % elseif x > xS(n) 
%     %     y = y*U(n, ind(i)); 
%     % end
% end

% end