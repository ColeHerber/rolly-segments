import numpy as np
import control

def get_inertia_values():
    """
    Extracts and adjusts inertia values based on user images.
    Returns values in Standard SI Units (kg*m^2).
    """
    # --- Constants for Unit Conversion ---
    # 1 g*mm^2 = 1e-3 kg * (1e-3 m)^2 = 1e-9 kg*m^2
    G_MM2_TO_KG_M2 = 1e-9
    
    # 1 g*cm^2 = 1e-3 kg * (1e-2 m)^2 = 1e-7 kg*m^2
    G_CM2_TO_KG_M2 = 1e-7
    
    # 1 lb*mm^2 = 0.45359237 kg * (1e-3 m)^2
    LB_MM2_TO_KG_M2 = 0.45359237 * 1e-6

    # --- Wheel Inertia (from "Balancer" image) ---
    # Name "Balancer" implies this is the rotating mass (Flywheel)
    lyy_wheel_raw = 450120.293 # g*mm^2
    I_w = lyy_wheel_raw * G_MM2_TO_KG_M2

    # --- Body Inertia (from "Assembly 2" image) ---
    # This image was in Imperial Units (lb*mm^2)
    # 1616.84 lb*mm^2 is approx 733,000 g*mm^2 (Consistent with previous Body data)
    lyy_body_imperial = 1616.84196 # lb*mm^2
    
    # Convert body base inertia to SI
    I_body_base = lyy_body_imperial * LB_MM2_TO_KG_M2
    
    # Add the requested 61 g*cm^2 offset to the Body
    offset_raw = 61.0 # g*cm^2
    offset_si = offset_raw * G_CM2_TO_KG_M2
    
    # Total Body Inertia
    I_b = I_body_base + offset_si

    return I_w, I_b

def solve_lqr_cubli(I_w, I_b):
    """
    Sets up the linearized dynamics for a Reaction Wheel Pendulum (Cubli)
    and solves for the LQR control gains.
    """
    print(f"--- System Parameters (SI Units) ---")
    print(f"Wheel Inertia (I_w): {I_w:.8f} kg*m^2")
    print(f"Body Inertia  (I_b): {I_b:.8f} kg*m^2")

    # --- MISSING PHYSICAL PARAMETERS ---
    # User must update these values for the controller to work on real hardware
    m_total = 0.5   # [kg] Total mass of the system (Placeholder)
    l_com   = 0.08  # [m]  Distance from pivot point to Center of Mass (Placeholder)
    g       = 9.81  # [m/s^2] Gravity
    
    # Calculate Inertia about the pivot point using Parallel Axis Theorem
    # Assuming the CAD inertia extracted (I_b) was about the Center of Mass
    I_p = I_b + m_total * (l_com**2)
    
    print(f"Inertia @ Pivot (I_p): {I_p:.8f} kg*m^2 (Assumes I_b is about COM)")

    # --- Linearized State Space Model ---
    # State Vector x = [theta, theta_dot, psi_dot]
    # theta:     Body angle (rad)
    # theta_dot: Body angular velocity (rad/s)
    # psi_dot:   Wheel angular velocity (rad/s)
    
    # Equations of Motion (Linearized around vertical equilibrium):
    # 1. (I_p + I_w) * theta_ddot + I_w * psi_ddot - m*g*l * theta = 0
    # 2. I_w * (theta_ddot + psi_ddot) = Torque (T)
    
    # We rearrange to solve for theta_ddot and psi_ddot in terms of state and input T.
    # Determinant for mass matrix inversion
    # det = I_p * I_w
    
    # A Matrix (3x3)
    # x_dot = A * x + B * u
    # Rows: [theta_dot, theta_ddot, psi_ddot]
    
    A = np.zeros((3, 3))
    A[0, 1] = 1.0 # d(theta)/dt = theta_dot
    
    # Derived from dynamics derivation:
    # theta_ddot = (m*g*l / I_p) * theta + (1 / I_p) * T
    # psi_ddot   = -(m*g*l / I_p) * theta - ((I_p + I_w) / (I_p * I_w)) * T
    
    # However, strictly speaking, reaction torque acts opposite on body. 
    # Standard Reaction Wheel Pendulum Dynamics:
    # (I_p) * theta_ddot = m*g*l*sin(theta) - T
    # I_w * (psi_ddot + theta_ddot) = T  => psi_ddot = T/I_w - theta_ddot
    
    # Linearizing:
    # theta_ddot = (m*g*l/I_p)*theta - (1/I_p)*T
    # psi_ddot   = (T/I_w) - ((m*g*l/I_p)*theta - (1/I_p)*T)
    # psi_ddot   = -(m*g*l/I_p)*theta + (1/I_w + 1/I_p)*T
    
    A[1, 0] = (m_total * g * l_com) / I_p
    A[2, 0] = -(m_total * g * l_com) / I_p
    
    # B Matrix (3x1)
    B = np.zeros((3, 1))
    B[1, 0] = -1.0 / I_p
    B[2, 0] = (1.0 / I_w) + (1.0 / I_p)

    print("\n--- State Space Matrices ---")
    print("A Matrix:\n", A)
    print("B Matrix:\n", B)

    # --- LQR Design ---
    # Q: State Cost Matrix (Penalize error)
    # Tuning priority: Keep theta close to 0, limit wheel speed
    Q = np.diag([100.0,  # Penalty on Body Angle (theta)
                 1.0,    # Penalty on Body Velocity (theta_dot)
                 0.01])  # Penalty on Wheel Velocity (psi_dot)

    # R: Input Cost Matrix (Penalize control effort/Torque)
    R = np.array([[10.0]])

    # Calculate LQR Gain using python-control library
    # Returns: K (Gain), S (Riccati Solution), E (Eigenvalues)
    K, S, E = control.lqr(A, B, Q, R)

    return K

if __name__ == "__main__":
    # 1. Get Inertias
    I_wheel, I_body = get_inertia_values()
    
    # 2. Solve Control
    K = solve_lqr_cubli(I_wheel, I_body)
    
    # 3. Motor Parameters
    Kt = 0.115  # Torque Constant [Nm/A]

    print("\n--- Control Output (Torque) ---")
    print(f"LQR Gains K = {K}")
    print(f"u(t) = - ({K[0,0]:.4f} * theta) - ({K[0,1]:.4f} * theta_dot) - ({K[0,2]:.4f} * wheel_velocity)")
    
    # 4. Convert to Current (Amps) for Motor Driver
    # Since Torque T = Kt * I, then I = T / Kt
    # We divide the Torque gains by Kt to get Current gains
    K_amps = K / Kt
    
    print("\n--- Control Output (Current / Amps) ---")
    print(f"Motor Torque Constant (Kt): {Kt} Nm/A")
    print(f"Current Gains K_amps = {K_amps}")
    print("\nFINAL CONTROL LAW (Send this I to motor):")
    print(f"I(t) = - ({K_amps[0,0]:.4f} * theta) - ({K_amps[0,1]:.4f} * theta_dot) - ({K_amps[0,2]:.4f} * wheel_velocity)")