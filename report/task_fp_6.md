The data collected for the camera ttc report can be found in `table.csv`.
This data was generated using the executable `print_table`, which is one of the build targets of the c++ project.

The data in the table was used by the script `plots.py` to generate plots.
Check out the plots in the directory `plots/`.

Each plot shows the ttc computed using lidar and camera using a particular detector-descriptor setup.
(Indeed, it's redundant to show the lidar ttc for each setup, since they are not affected by keypoints. But whatever.)

Each plot also shows a curve of expected TTCs. These were computed "posthoc" by assuming that the car in front of the
ego vehicle has a constant velocity throughout the entire 1.7s period. See `task_fp_5.md` for details on that.

Some observations from the plots

- The HARRIS detector had consistently bad performance, with wildly fluctuating ttc values. The TTC algorithm I wrote is probably just as much to blame as the detector itself.
- The AKAZE detector and descriptor combination seemed to have the nicest performance with the present TTC algorithm.
- Most of the camera-based TTC estimates *underestimated* the TTC. One possible contributor to this is that the camera was closer to the vehicle by 0.27m compare to the lidar. So the posthoc estimate, which was based on lidar, is a little greater (by about half a second, it turns out) than the time it would take for the *camera* lens to collide with the vehicle at constant velocity.
- 
