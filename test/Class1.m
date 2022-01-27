classdef Class1
    %CLASS1 Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        A1
    end
    
    methods
        function obj = Class1(a1)
            %CLASS1 Construct an instance of this class
            %   Detailed explanation goes here
            obj.A1 = a1;
        end
        
        function outputArg = method1(obj,inputArg)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            outputArg = obj.Property1 + inputArg;
        end
    end
end

