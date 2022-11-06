classdef Class1
    %CLASS1 Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        a
    end
    
    methods
        function obj = Class1()
            obj.a = 1;
        end
        
        function Print(obj)
            disp('Class 1')
        end

        function sobj = saveobj(obj)
            disp('saveobj()')
        end
    end
end

