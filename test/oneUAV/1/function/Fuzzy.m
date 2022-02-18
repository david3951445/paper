classdef Fuzzy
    %T-S fuzzy model
    
    properties (Constant)
        % operation points
        OP = {
            linspace(-0.5, 0.5, 4)
            linspace(-0.5, 0.5, 4)
            [pi/2, 0]
        };

        % PV = [7, 9] % loction of chosen premise variable in state x
        PV = [7, 9, 11]
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
            % len_OP
            obj.len_OP = length(obj.OP);

            % op
            for i = 1 : obj.len_OP
                obj.op(i).val = obj.OP{i};
                obj.op(i).len = length(obj.OP{i});
            end
            
            % num
            obj.num = 1;
            for i = 1 : obj.len_OP
                obj.num = obj.num*obj.op(i).len;
            end   
            
            % set, type 
            ind = 1;    
            for i1 = 1 : obj.op(1).len
                for i2 = 1 : obj.op(2).len
                    for i3 = 1 : obj.op(3).len
                        % obj.set(:, ind) = [obj.op(1).val(i1); obj.op(2).val(i2)];
                        % obj.type(:, ind) = [i1; i2];

                        obj.set(:, ind) = [obj.op(1).val(i1); obj.op(2).val(i2); obj.op(3).val(i3)];
                        obj.type(:, ind) = [i1; i2; i3];
                        ind = ind + 1;
                    end
                end
            end
            
        end
        
        function y = mbfun(obj, k, x) % membership function of rule k
            y = 1;
            for i = 1 : obj.len_OP
                y = y*grade(obj, k, x(obj.PV(i)), i);
            end
        end
        
        % function y = alpha(obj, k, x) % Interpolation function
        %     e = 10^(-10); % prevent dividing by 0
        %     num = 1/(norm(x(obj.PV) - obj.set(k))+e);
        %     den = 0;
        %     for i = 1 : obj.num
        %         den = den + 1/(norm(x(obj.PV) - obj.set(i))+e);
        %     end

        %     y = num/den;       
        % end
    end
    
    methods (Access = private)
        % grade of membership(triangular function). old method
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
%#ok<*PROPLC>