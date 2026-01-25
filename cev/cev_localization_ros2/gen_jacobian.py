import sympy as sp

# Define state variables
x, y, z = sp.symbols('x y z')
roll, pitch, yaw = sp.symbols('roll pitch yaw')
dx, dy, dz = sp.symbols('dx dy dz')
droll, dpitch, dyaw = sp.symbols('droll dpitch dyaw')
ddx, ddy, ddz = sp.symbols('ddx ddy ddz')
tau, dtau, ddtau = sp.symbols('tau dtau ddtau')
dt, wheelbase = sp.symbols('dt wheelbase')

# New state definitions
new_x = x + dx * sp.cos(yaw + tau) * dt
new_y = y + dx * sp.sin(yaw + tau) * dt
new_dx = dx + ddx * dt
new_dy = dy + ddy * dt
new_dtau = dtau + ddtau * dt
new_dyaw = dx * sp.sin(tau) / wheelbase * dt
new_yaw = yaw + new_dyaw
new_tau = tau + dtau * dt

# Combine into vectors
new_state = sp.Matrix([
    new_x, new_y, z, roll, pitch, new_yaw,
    new_dx, new_dy, dz, droll, dpitch, new_dyaw,
    ddx, ddy, ddz, new_tau, new_dtau, ddtau
])

current_state = sp.Matrix([
    x, y, z, roll, pitch, yaw,
    dx, dy, dz, droll, dpitch, dyaw,
    ddx, ddy, ddz, tau, dtau, ddtau
])

# Compute the Jacobian
jacobian = new_state.jacobian(current_state)

# State names for mapping
state_names = [
    "x__", "y__", "z__", "roll__", "pitch__", "yaw__",
    "d_x__", "d_y__", "d_z__", "d_roll__", "d_pitch__", "d_yaw__",
    "d2_x__", "d2_y__", "d2_z__", "tau__", "d_tau__", "d2_tau__"
]

# Generate C++ code for non-zero Jacobian entries
cpp_code = []
for i in range(jacobian.shape[0]):
    for j in range(jacobian.shape[1]):
        entry = jacobian[i, j]
        if entry not in [0, 1]:  # Skip zero entries
            cpp_code.append(f"F_k({state_names[i]}, {state_names[j]}) = {sp.ccode(entry)};")

# Print the generated C++ code
print("\n".join(cpp_code))
