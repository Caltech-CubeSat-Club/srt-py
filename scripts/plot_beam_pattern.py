import matplotlib.pyplot as plt
import numpy as np
from scipy.special import j1

lambda_meters = 0.21
aperture_meters = 6.0

theta_steps, phi_steps = 100, 100
theta = np.linspace(0, 2.5, theta_steps) # 2.5 degrees radius
phi = np.linspace(0, 2 * np.pi, phi_steps)

R, Th = np.meshgrid(theta, phi)
E = (2 * lambda_meters / np.pi / aperture_meters) * j1((np.pi * aperture_meters / lambda_meters) * np.sin(np.radians(R))) / np.sin(np.radians(R))

fig, ax = plt.subplots(subplot_kw={'projection': 'polar'})
mesh = ax.pcolormesh(Th, R, E, cmap='gray', shading='auto', antialiased=False)
ax.set_axis_off()
plt.savefig('beam_pattern.png', bbox_inches='tight', pad_inches=0, dpi=300, transparent=True)