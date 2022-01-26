import numpy as np
pc_batch =[]
pc_ori = np.zeros((1024, 3))
indices = np.arange(0, len(pc_ori))
if len(pc_ori) > 1024:
    choice = np.random.choice(indices, size=1024, replace=True)
else:
    choice = np.random.choice(indices, size=1024, replace=False)

pc_ds = pc_ori[choice]
pc_batch.extend([pc_ds,pc_ds,pc_ds,pc_ds,pc_ds,pc_ds,pc_ds,pc_ds,pc_ds,pc_ds,pc_ds,pc_ds,
                 pc_ds,pc_ds,pc_ds,pc_ds,pc_ds,pc_ds,pc_ds,pc_ds,pc_ds,pc_ds,pc_ds,pc_ds,
                 pc_ds,pc_ds,pc_ds,pc_ds,pc_ds,pc_ds,pc_ds,pc_ds])
pc_batch_np = np.asarray(pc_batch, dtype=np.float32)
print(pc_batch_np.shape)