import numpy as np

# Old value of prism
Prism_1_old = np.array([-0.11493461, -0.32969205, 0.33997508])
Prism_2_old = np.array([-1.04115493, 0.00147775, 0.23972586])
Prism_3_old = np.array([-0.29393989, 0.32581738, 0.28939597])

Prism_1 = np.array([-0.12413052, -0.32998385, 0.34745009])
Prism_2 = np.array([-1.05061716, -0.00439399, 0.2448696])
Prism_3 = np.array([-0.30736107, 0.32429536, 0.30031949])
Dist_prism_12 = np.linalg.norm(Prism_1-Prism_2)*1000
Dist_prism_13 = np.linalg.norm(Prism_1-Prism_3)*1000
Dist_prism_23 = np.linalg.norm(Prism_2-Prism_3)*1000
