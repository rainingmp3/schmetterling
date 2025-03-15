import control as ctrl
import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import solve_ivp
from simple_pid import PID


m = 1
M = 5
L = 2
g = -9.8
d = 1
b = 1 # pendulum up phi = 180

dt = 0.01
u = 0

A = np.array([[0, 1, 0, 0],
              [0, -d/M, b*m*g/M, 0],
              [0, 0, 0, 1],
              [0, -b*d/(M*L), -b*g*(M + m)/(M*L),  0]])

B = np.array([[0],
              [1/M],
              [0],
              [b/(M*L)]])

C = np.array([[1, 0, 0, 0],
              [0, 1, 0, 0],
              [0, 0, 1, 0],
              [0, 0, 0, 1]])

D = np.array([[0],
              [0],
              [0],
              [0]])

S = ctrl.ss(A, B, C, D)
print('S = ', S)


def integr_non_lin(t, y, u):
    Sy = np.sin(y[2])
    Cy = np.cos(y[2])
    D = m * L * L * (M + m * (1 - Cy**2))

    int_res = [0, 0, 0, 0]

    int_res[0] = y[1]
    int_res[1] = (1 / D) * (-m**2 * L**2 * g * Cy * Sy + m * L**2 * (m * L * y[3]**2 * Sy - d * y[1])) + m * L * L * (1 / D) * u
    int_res[2] = y[3]
    int_res[3] = (1/D)*((m+M)*m*g*L*Sy - m*L*Cy*(m*L*y[3]**2*Sy - d*y[1])) - m*L*Cy*(1/D)*u
    return int_res
t = 0  # [s]
t_end = 10  # [s]
u = 0

sim_results = []
T_sim = [t, t+dt]

x = [-1, 0, np.pi + 0.1, 0]

res = {'time': [],
       'command': [],
       'y_position': [],
       'y_angle': [],
       'x_position': [],
       'x_velocity': [],
       'x_angle': [],
       'x_angular_velocity': []
       }

res['time'].append(T_sim[0])
res['command'].append(u)
res['y_position'].append(x[0])
res['y_angle'].append(x[2])
res['x_position'].append(x[0])
res['x_velocity'].append(x[1])
res['x_angle'].append(x[2])
res['x_angular_velocity'].append(x[3])

setpoint=np.deg2rad(180)
pid = PID(100, 1, 20, setpoint=setpoint, sample_time=dt)

while t < t_end:
    sol = solve_ivp(integr_non_lin, T_sim, y0=x, args =(u,),)

    u = pid(sol.y[2][1], dt)
  #  u = 0

    x = [sol.y[0][1], sol.y[1][1], sol.y[2][1], sol.y[3][1]]
    
    res['time'].append(T_sim[-1])
    res['command'].append(u)
    res['x_position'].append(sol.y[0][1])
    res['x_velocity'].append(sol.y[1][1])
    res['x_angle'].append(sol.y[2][1])
    res['x_angular_velocity'].append(sol.y[3][1])

    T_sim = [t, t + dt]
    t = t + dt

plt.subplot(5, 1, 1)
plt.plot(res['time'], res['x_position'])
plt.xlabel('t')
plt.ylabel('y_position')
plt.grid()

plt.subplot(5, 1, 2)
plt.plot(res['time'], res['x_velocity'])
plt.xlabel('t')
plt.ylabel('x_velocity')
plt.grid()

plt.subplot(5, 1, 3)
plt.step(res['time'], res['x_angle'], where='post')
plt.xlabel('t')
plt.ylabel('y_angle')
plt.grid()

plt.subplot(5, 1, 4)
plt.step(res['time'], res['x_angular_velocity'], where='post')
plt.xlabel('t')
plt.ylabel('x_angular_velocity')
plt.grid()

plt.subplot(5, 1, 5)
plt.step(res['time'], res['command'], where='post')
plt.xlabel('t')
plt.ylabel('command')
plt.grid()
plt.show()
