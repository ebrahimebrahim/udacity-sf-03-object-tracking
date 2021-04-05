import matplotlib.pyplot as plt
import pandas as pd
import numpy as np

df = pd.read_csv('table.csv')

det_desc_pairs = df.detector.combine(df.descriptor, lambda a,b: (a,b)).unique()

frame=np.arange(1.,19.,1.)
t = (frame-1.0) / 17.0
ttc_posthoc = 10.9*t + 12.6*(1.0-t)


for detector,descriptor in det_desc_pairs:
  plt.clf()
  this_df = df[(df.descriptor==descriptor) & (df.detector == detector)]
  plt.plot(this_df.frameindex, this_df.ttc_camera, 'ro', label='camera')
  plt.plot(this_df.frameindex, this_df.ttc_lidar, 'go', label='lidar')
  plt.plot(frame, ttc_posthoc, label='posthoc estimate')

  plt.xlim((1,18))
  plt.xlabel('frame number')
  plt.ylabel('time to collision (s)')
  plt.ylim((0,16))
  plt.legend()

  plt.title(detector + ' detector and ' + descriptor + ' descriptor')
  plt.savefig('plots/'+detector+'_'+descriptor+'.png')
