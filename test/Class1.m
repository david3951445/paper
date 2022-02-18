classdef Class1
    %CLASS1 Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        A1
        B1 = 1
    end
    
    methods
        function obj = Class1(a1)
            %CLASS1 Construct an instance of this class
            %   Detailed explanation goes here
            obj.A1 = a1;
        end
        
        function obj = setA(obj, a1)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            obj.A1 = a1;
        end
    end
end

