import pickle
import numpy as np

with open('/home/lv-robotics/workspace/raw_data/corrected_data/0040/state.pkl', 'rb') as f:
    corrected = pickle.load(f)

with open('/home/lv-robotics/workspace/raw_data/egg_to_bowl_replayed/0040_v1/state.pkl', 'rb') as f:
    replayed = pickle.load(f)

for side in ('right_arm',):
    cg = np.array(corrected[side]['joints'])[:, 6]
    rg = np.array(replayed[side]['joints'])[:, 6]
    print(f'=== {side} gripper ===')
    print(f'corrected_data  n={len(cg)}  min={cg.min():.4f}  max={cg.max():.4f}')
    print(f'replayed        n={len(rg)}  min={rg.min():.4f}  max={rg.max():.4f}')
    print()
    print(f'corrected hold-phase frames (g==0.0): {int((cg == 0.0).sum())}')
    print(f'replayed  hold-phase frames (g==0.0): {int((rg == 0.0).sum())}')
    print()
    hold_mask = cg == 0.0
    print(f'replayed gripper values where corrected=0.0:')
    print(f'  min={rg[hold_mask].min():.4f}  max={rg[hold_mask].max():.4f}  mean={rg[hold_mask].mean():.4f}')
