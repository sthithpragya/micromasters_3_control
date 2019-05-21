% controller(2.5, [1, 1, pi/2])

t_stop = 18;
[t, robot_path] = ode45(@controller, [0 t_stop], [0 0 0]')