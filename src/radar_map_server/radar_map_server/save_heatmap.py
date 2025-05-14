# save_heatmap.py (run once)
import numpy as np

# Replace this with the actual nested list, or automate it with some other script
#   do as you please, but this is the format that the script is looking for
heatmap_list = [
#  [12.4,  0.0, …, 3.1],
#  [ 0.0, 45.2, …, 1.7],
#   …
]
arr = np.array(heatmap_list, dtype=float)
np.save('/home/you/maps/heatmap.npy', arr)
