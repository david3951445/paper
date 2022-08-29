function r3 = Post_processing(pp, r1, INTERP_DENSITY)
    r3 = fit_linear_spline(r1, pp.INTERP_DENSITY);
end