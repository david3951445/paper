classdef Fuzzy  
    properties (Constant)
        OP = {linspace(-pi/10, pi/10, 3), linspace(-pi/10, pi/10, 3)}; % operation points
        PV = [7, 9] % loction of chosen premise variable in state x
    end
    
    properties
        op      % operation points
        len_OP  % length of OP
        num     % number of rules 
        set     % set(:, i) : fuzzy set in fuzzy rule i
        type    % type(:, i) : mbfun's type in fuzzy rule i
    end
    
    methods
        function obj = Fuzzy() % initialize
            % op
            len = {length(cell2mat(obj.OP(1))), length(cell2mat(obj.OP(2)))};
            obj.op = struct('val',obj.OP, 'len', len);
            
            % len_OP
            obj.len_OP = length(obj.OP);
            
            % num
            obj.num = 1;
            for i = 1 : obj.len_OP
                obj.num = obj.num*obj.op(i).len;
            end   
            
            % set, type     
            for i = 1 : obj.op(1).len
                for j = 1 : obj.op(2).len
                    obj.set(:, (i-1)*obj.op(2).len + j) = [obj.op(1).val(i); obj.op(2).val(j)];
                    obj.type(:, (i-1)*obj.op(2).len + j) = [i; j];
                end
            end
            
        end
        
        function y = mbfun(obj, k, x) % membership function of rule k
            total = 1;
            for i = 1 : obj.len_OP
                total = total*grade(obj, k, x(obj.PV(i)), i);
            end
            y = total;
        end
        
        function y = alpha(obj, k, x) % Interpolation function
            e = 10^(-10); % prevent dividing by 0
            num = 1/(norm(x(obj.PV) - obj.set(k))+e);
            den = 0;
            for i = 1 : obj.num
                den = den + 1/(norm(x(obj.PV) - obj.set(i))+e);
            end

            y = num/den;
            end
    end
    
    methods (Access = private)
        % grade of membership(triangular function)
        function y = grade(obj, k, x, i_op)  
            V = obj.op(i_op).val;
            L = obj.op(i_op).len;
            i = obj.type(i_op, k);
            
            switch i
                case 1 % z-shaped ""\__
                    if x <= V(i)
                        y = 1;
                    elseif x > V(i) && x <= V(i+1)
                        y = 1 - (x - V(i))/(V(i+1)-V(i));
                    else %if x >= V(i+1)
                        y = 0;
                    end
                    
                case L % s-shaped __/""
                    if x <= V(i-1)
                        y = 0;
                    elseif x > V(i-1) && x <= V(i)
                        y = 1 - (x - V(i))/(V(i-1)-V(i));
                    else %if x >= V(i)
                        y = 1;
                    end
                    
                otherwise % triangle _/\_
                    if x <= V(i-1)
                        y = 0;
                    elseif x > V(i-1) && x <= V(i)
                        y = 1 - (x - V(i))/(V(i-1)-V(i));
                    elseif x > V(i) && x <= V(i+1)
                        y = 1 - (x - V(i))/(V(i+1)-V(i));
                    else %if x >= V(i+1)
                        y = 0;
                    end
            end
        end
    end
end

