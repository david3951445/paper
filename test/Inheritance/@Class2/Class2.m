classdef Class2 < Class1
    %CLASS1 Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        % A1
        C1
    end
    
    methods
        function obj = Class2(c1)
            %CLASS1 Construct an instance of this class
            %   Detailed explanation goes here
            obj = obj@Class1(3);
            obj.C1 = 2;
        end
        
        function obj = setC(obj, a1)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            obj.C1 = a1;
        end
        function Print(obj)
            obj.Print@Class1();
            2
        end
    end
end

