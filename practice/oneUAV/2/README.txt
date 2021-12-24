1. What's this?
version 2 of oneUAV.
update :
    - UAV.m : the uav model f(), g() is changed to [1], but not exactly the same.
    - REF.m : r()'s y(7), y(9) are changed. The idea is to let y(7), y(9) match y(1)~y(3).
    - trajectory.m : add Euler method, feedfoward term uo.
    - oneUAV.m : add inverse dynamic
    - getControlGain : changed A to 0.01*I, F to [I A; O Br]

2. ref
[1] https://www.kth.se/polopoly_fs/1.588039.1600688317!/Thesis%20KTH%20-%20Francesco%20Sabatino.pdf
