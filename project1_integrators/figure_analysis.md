Figure 1 — Trajectory Comparisons (RK4 vs Exact)
What it shows: Whether the numerical integrator produces an accurate solution compared to the known mathematical truth.

It has two side-by-side subplots, each solving a different ODE:

Left panel — Exponential Decay
ODE: 
𝑦
˙
=
−
2
𝑦
y
˙
​
 =−2y, starting at 
𝑦
(
0
)
=
1
y(0)=1
Exact solution: 
𝑦
(
𝑡
)
=
𝑒
−
2
𝑡
y(t)=e 
−2t
  — a smooth curve that decays from 1 toward 0
What's plotted: The RK4 numerical solution (solid line, dt=0.01) overlaid on the exact analytical solution (red dashed line)
What to look for: The two lines should be nearly indistinguishable, confirming RK4 is highly accurate on this problem
Right panel — Harmonic Oscillator
ODE: 
𝑥
¨
=
−
4
𝑥
x
¨
 =−4x, i.e. a spring with 
𝜔
=
2
ω=2, starting at 
𝑥
(
0
)
=
1
x(0)=1, 
𝑥
˙
(
0
)
=
0
x
˙
 (0)=0
Exact solution: 
𝑥
(
𝑡
)
=
cos
⁡
(
2
𝑡
)
x(t)=cos(2t) — a sinusoid that oscillates forever without decay
What's plotted: Same idea — RK4 numerical position vs exact cosine
What to look for: The numerical solution should track the oscillation without drifting or growing over time, showing that RK4 conserves energy well
Reasoning: These are chosen because their exact solutions are known, so you can directly measure correctness. Exponential decay tests monotone behavior; harmonic oscillator tests periodic behavior and energy conservation.

Figure 2 — Convergence Analysis (log-log)
What it shows: How quickly each method's error shrinks as you use a smaller step size dt. This verifies the theoretical order of accuracy of each integrator.

It again has two subplots (same two ODEs), each plotting error vs step size on a log-log scale.

Three methods compared:
Method	Expected order	Meaning
Euler	
𝑂
(
ℎ
1
)
O(h 
1
 )	Halve dt → error halves
RK2	
𝑂
(
ℎ
2
)
O(h 
2
 )	Halve dt → error quarters
RK4	
𝑂
(
ℎ
4
)
O(h 
4
 )	Halve dt → error drops 16×
How to read the log-log plot:
The x-axis is step size dt (from large to small), y-axis is global error at t_end
On a log-log plot, a method with order 
𝑝
p appears as a straight line with slope 
𝑝
p
The black reference lines (dotted = slope 1, dashed = slope 2, solid = slope 4) are the theoretical ideal slopes
What to look for: Each colored line should run parallel to its matching reference slope. If RK4's line is parallel to "slope 4", the implementation is correct. If a line is shallower than expected, the method has a bug or a precision ceiling has been hit.

Reasoning: This is the standard way numerical analysts verify an integrator — you don't just check one answer, you check that accuracy improves at the right rate as dt decreases. A correct result at one step size could be a coincidence; the right convergence slope cannot.