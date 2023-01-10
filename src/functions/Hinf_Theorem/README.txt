Solving control gain K and observer gain L using H infinity theorem

- K : Solving control gain
- K_L : Solving control and observer gain
- L : Solving observer gain

- Different between solveLMI1(), ..., solveLMIn()
	- Choice of Lyapunov function V = x*Pb*x
		Pb = [P1 0; 0 P2] or [P 0; 0 P] or [P -P; -P P] ...
	- Choice of augment state
		xb = [x; xr] or [x; x-xr] ...
	- Weighting matrix
		Qb = [Q1 0; 0 Q2] or [Q -Q; -Q Q]
	- Note: The corresponding Ab, Bb, Kb may change too.
- Recommend solveLMI1(), solveLMI10().