"""
Simulate and plot the two gyroscope integration error types:
  1. Bias drift  (linear growth)
  2. Random walk (sqrt growth)
Also shows their combined effect and a comparison at two sampling rates.
"""

import numpy as np
import matplotlib.pyplot as plt
import matplotlib
matplotlib.use('Agg')  # non-interactive backend
matplotlib.rcParams.update({
    'font.size': 10,
    'font.family': 'serif',
    'figure.dpi': 200,
    'axes.grid': True,
    'grid.alpha': 0.3,
})

np.random.seed(42)

# --- Parameters ---
T = 30.0           # total time (seconds)
fs = 100            # sampling frequency (Hz)
dt = 1.0 / fs
N = int(T / dt)
t = np.linspace(0, T, N)

b_omega = 0.05      # bias (deg/s) — typical MEMS gyroscope
sigma_omega = 0.1   # noise std dev (deg/s)

# --- Simulate errors ---
bias_drift = b_omega * t                                    # linear
noise_samples = sigma_omega * np.random.randn(N)
random_walk = np.cumsum(noise_samples) * dt                 # sqrt growth
combined = bias_drift + random_walk

# Theoretical envelope for random walk (1-sigma)
# Var[epsilon_rw] = sigma^2 * dt^2 * N = sigma^2 * dt * T
# std[epsilon_rw] = sigma * sqrt(dt * T)
rw_theory = sigma_omega * np.sqrt(dt * t)  # correct 1-sigma envelope

# --- Figure 1: Three error types ---
fig, axes = plt.subplots(3, 1, figsize=(6, 7), sharex=True)

# Panel 1: Bias drift
axes[0].plot(t, bias_drift, color='#c0392b', linewidth=1.5)
axes[0].set_ylabel('Error (deg)')
axes[0].set_title('(a) Bias Drift:  $\\epsilon_{bias} = b_\\omega \\cdot T$  (linear growth)',
                   fontsize=9, loc='left')
axes[0].annotate(f'$b_\\omega = {b_omega}$ deg/s',
                 xy=(T*0.7, b_omega*T*0.7), fontsize=9,
                 color='#c0392b')

# Panel 2: Random walk
axes[1].plot(t, random_walk, color='#2980b9', linewidth=0.8, alpha=0.8,
             label='Simulated random walk')
axes[1].plot(t, rw_theory, '--', color='#2c3e50', linewidth=1.2,
             label='$\\pm\\sigma_\\omega\\sqrt{\\Delta t \\cdot T}$ envelope (1$\\sigma$)')
axes[1].plot(t, -rw_theory, '--', color='#2c3e50', linewidth=1.2)
axes[1].set_ylabel('Error (deg)')
axes[1].set_title('(b) Random Walk:  $\\sigma_{rw} = \\sigma_\\omega\\sqrt{\\Delta t \\cdot T}$  (sublinear growth)',
                   fontsize=9, loc='left')
axes[1].legend(fontsize=8, loc='upper left')

# Panel 3: Combined
axes[2].plot(t, combined, color='#8e44ad', linewidth=1.0, alpha=0.9,
             label='Combined error')
axes[2].plot(t, bias_drift, '--', color='#c0392b', linewidth=1.0, alpha=0.6,
             label='Bias drift only')
axes[2].set_ylabel('Error (deg)')
axes[2].set_xlabel('Time (s)')
axes[2].set_title('(c) Combined:  bias drift dominates at long times',
                   fontsize=9, loc='left')
axes[2].legend(fontsize=8, loc='upper left')

plt.tight_layout()
plt.savefig('../images/gyro_drift_errors.pdf', bbox_inches='tight')
plt.savefig('../images/gyro_drift_errors.png', bbox_inches='tight')
print("Saved: gyro_drift_errors.pdf / .png")


# --- Figure 2: Sampling frequency comparison ---
fig2, axes2 = plt.subplots(2, 1, figsize=(6, 5), sharex=True)

for i, (fs_i, color, ls) in enumerate([(50, '#e74c3c', '-'), (200, '#27ae60', '-')]):
    dt_i = 1.0 / fs_i
    N_i = int(T / dt_i)
    t_i = np.linspace(0, T, N_i)

    # True signal: a smooth rotation (sinusoidal angular velocity)
    omega_true = 10 * np.sin(2 * np.pi * 0.5 * t_i)  # deg/s
    theta_true = np.cumsum(omega_true) * dt_i

    # With bias + noise
    omega_meas = omega_true + b_omega + sigma_omega * np.random.randn(N_i)
    theta_meas = np.cumsum(omega_meas) * dt_i

    error = theta_meas - theta_true

    axes2[0].plot(t_i, theta_true, color='#2c3e50', linewidth=1.5,
                  label='Ground truth' if i == 0 else None)
    axes2[0].plot(t_i, theta_meas, color=color, linewidth=0.8, alpha=0.8,
                  label=f'Gyro integrated ({fs_i} Hz)')

    axes2[1].plot(t_i, error, color=color, linewidth=0.8, alpha=0.8,
                  label=f'{fs_i} Hz  (error at T={int(T)}s: {error[-1]:.2f} deg)')

axes2[0].set_ylabel('Angle (deg)')
axes2[0].set_title('Gyroscope-integrated angle vs ground truth', fontsize=9, loc='left')
axes2[0].legend(fontsize=8)

axes2[1].set_ylabel('Error (deg)')
axes2[1].set_xlabel('Time (s)')
axes2[1].set_title('Accumulated error: bias drift dominates regardless of sampling rate',
                    fontsize=9, loc='left')
axes2[1].legend(fontsize=8)

plt.tight_layout()
plt.savefig('../images/gyro_sampling_comparison.pdf', bbox_inches='tight')
plt.savefig('../images/gyro_sampling_comparison.png', bbox_inches='tight')
print("Saved: gyro_sampling_comparison.pdf / .png")

print("Done.")
