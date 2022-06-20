# MPC_matlab
MPC understanding:<br />
the key point is to find an iteration equation x(i+1)/x(i) propagated by inputs/controls u(i), then optimize the object function J formated by (Xref-X)^2 quadrutic cost functions

hyperparameter: <br />
ts --- steps time    --- smaller means more precise similar to reference<br />
T  --- time horizon  --- larger means more early to expect changeing

bicycle cant do side drift that is because of the non-holonomic constraint<br />

diff_car_00: the original diff car model from pranav's robotic class (https://www.youtube.com/watch?v=n8Tda3KFvZc&t=2537s)<br />
diff_car_01: the diff car model without non-holonomic constraints so that vx, vy, omega are independent to each other
