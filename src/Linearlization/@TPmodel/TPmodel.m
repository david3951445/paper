classdef TPmodel
    %TP type polytopic LPV model (transformed from LPV model)
    
    properties (Access = public)
        val         % Core tensor (or "linear matrice")
        len         % number of Core tensor (length(val))
        index       % for indexing "mf_discrete"
        mf_discrete % Discrete (because it's composed by "points") membership function (or "weighting functions") of Core tensor
    end

    properties (Access = private)
        sizeO       % Origin size in hosvd result, just for indexing M.mf(). ex: size(S) = [9, 2, 9, 2, 4, 4] -> A.sizeO = size(S, 1 : 4)
    end
    
    methods
        function o = TPmodel()
        end
        
        y = mf(o, x, ind) % membership function
        o = getTPmodel(o, para) % Execute TPmodel transformation
    end
end

