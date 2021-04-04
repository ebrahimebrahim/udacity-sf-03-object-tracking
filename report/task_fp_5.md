Let's use the lidar top view to estimate by hand the time to collision.
Then we can compare this with the result from frame-by-frame analysis of the program.

Look at the included screenshots `lidar_frame_1.png` and `lidar_frame_18.png`.
Assuming that the vehicle in front of the ego vehicle is not accelerating like crazy,
the average velocity over this 1.7s block of time should tell us what is a reasonable
neighborhood of values for the lidar TTC estimate in each frame.

In `lidar_frame_1.png` we see that the back of the vehicle has x coordinate approximately `x1 = 7.875 m`.

In `lidar_frame_18.png` we see that the x coordinate is approximately `x2 = 6.8125 m`.

This occurs over 17 frames, which is `dt = 1.7 s`.

The average relative velocity of the vehicle over this interval is `(x2-x1) / dt = - 0.625 m/s`.

If the car in front had a constant relative velocity of `v=0.625 m/s`, the TTC at frame 1 would be `-x1/v = 12.6s`
and the TTC at frame 18 would be `-x2/v = 10.9s`.

At the frames in between, a linear interpolation between 12.6s and 10.9s provides an estimate for what would be reasonable.
We have displayed this interpolated value alongside the lidar and camera TTC estimates in the program's camera image display.

Here are a couple of frames where the lidar-based TTC estimate is way off:

- Frame 6: See `bad_ttc_frame_6.png`. 

- Frame 7: See `bad_ttc_frame_7.png`.


The issue with both frames can be blamed on a cluster of lidar point that pops up a little closer than they should at frame 6. See `bad_cluster.png`.
There are enough of these points that the attempt by my algorithm to ignore outliers fails and places the car closer at frame 6 than it really must be.
Hence the low TTC estimate at frame 6 and the high TTC estimate at frame 7.

I don't know what about lidars makes such outlier clusters possible.
Maybe it's a timing issue? When we are told "these points came from frame 6" then 
as far I can tell they could have come from the beginning of the frame 6 interval or from the end of the interval.
The lidar spins to collect points, so there's no way for all the points to perfectly simultaneous.
Maybe that chunk of points on the car in front comes from a different segement of the frame interval from frame to frame.

I think a bit of smoothing of the signal across frames could fix this issue.
(i.e. consider the probability that the car in front changed its velocity so drastically in a tenth of a second, versus
the probability that the signal is noisy).
