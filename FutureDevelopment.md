# Introduction #

Some (but not all) of these directions for future development I intend to pursue myself. If anyone else would like to contribute to any items on this list, or make additional suggestions, this would be very welcome!


# Future Software Development #

  * **Live capture mode**
    * Just USB support should be provided. In my experience, many cameras (especially thermal cameras) that don't follow the UVC protocol exactly can be so peculiar that it really isn't worth trying to accommodate for them.
  * **Online occupancy map**
    * A dynamic display to the user of how well the FOV has been covered, and how good quality the sequence is so far (so that they know when to stop obtaining more footage).
  * **Port project to MacOS**
    * This should be pretty easy since MacOS is also unix-based.
  * **Port project to Windows**
    * Since OpenCV is cross-platform, the main issues should simply be argument parsing and file I/O.
  * **Integrate with a [ROS](http://www.ros.org/wiki/) package**
    * E.g perhaps [this](http://www.ros.org/wiki/camera_calibration) one? At present it seems to only support the standard pattern, and also the process of calibration is very slow, which I suspect is because they use the entire calibration sequence rather than a near-optimal subset. In fact there seems to be discussion about integrating [another pattern finding algorithm](http://answers.ros.org/question/1713/opencv-circles-grid-calibration) into the package.

# Future Algorithm Development #
  * **Improve Shape filter**
    * How about using the ratio of principal axis lengths of an elliptic fit / covariance matrix, rather than just the height and width of a bounding box which is always aligned with the image boundaries?
  * **Local acutance-based point rejection**
    * Some kind of local acutance test could be implemented around each corner, to see whether the individual corner has actually been sub-pixel located accurately. If not, the point could be removed from the 2D vector and 3D vector for that frame, so that it is not used for calibration and also not used for calculating ERE.
  * **Region-weighted projection error**
    * In some cases, accuracy over the entire FOV may be very important, or alternatively, only accuracy in the center may be of interest. The user should be able to provide or select from a number of "weighting" maps which can be used to optimize the model so that it is accurate over this area. At the moment, biases in the regions of the FOV given a large amount of attention in the provided footage can still lead to biases in the accuracy of the final model.
  * **Add 3+ camera calibration support**
    * A good approach may be to implement “stereoCalibrate()” between all pairs of cameras, and use some kind of bundle adjustment to achieve the best result. At the same time, I think removing the requirement that a pattern is visible by all cameras simultaneously would be very useful. This is not a huge problem for stereo configurations, but would become prohibitive for many other camera configurations.
  * **Wide-baseline camera setup support**
    * The intention of the triangle on the provided mask template was for to provide absolute referencing for this purpose, but it did not need to be implemented for the experiments outlined in the paper (and so it wasn't!).
  * **Add support for high-distortion lenses**
    * The system hasn't been tested on cameras with very severe distortion such as fish-eye lenses etc. Some changes may need to be made to accommodate this, such as adding more distortion models (hemispherical, fish-eye etc..)
  * **Automatically estimate pattern arrangement (MxN)**
    * Once the filtered MSERs are obtained, a number of "hypotheses" could be generated (e.g. if 40 squares are found, then hypotheses could include 4x10, 10x4, 5x8, 8x5, 6x6 etc). The most likely hypothesis based on a few frames could then be selected as the dimensions. The user would be prompted if the algorithm suspected it had failed in correctly estimating the pattern dimensions.
  * **Add a frame quality measure**
    * May need to first determine the characteristics of an image that make it good for accurate calibration (both individually and as part of a group). This could include checking the total accutance,  and the presence of motion blur.
  * **Improve Enhanced-MCM algorithm for automatic frame selection**
    * At the moment the algorithm assumes that the best “next” frame for N-frame calibration will ultimately help to make the best set. Occasionally I suspect that this is not the case. This is based on the fact that when using very similar sequences, occasionally the extended MRE is inflated a bit (~30%), and that the focal length can vary considerably (~10%).
  * **Add a centroid-only mode**
    * This would use only patch centroids - similarly to the OpenCV pattern-finding algorithm which uses circle-centers. This mode could be better for difficult cameras where the corners are not in focus, and perhaps would work well for extrinsics in many cases.
  * **Soft-synchronization option for cameras**
    * For extrinsic calibration, motion estimation could be used for each camera to reduce or even eliminate the need for synchronization. 3D motion tracks would need to be aligned, or something like that...
  * **Auto-calibration option using Manhattan assumption**
    * Just an idea I had.. could frequency analysis of the images be used to help? Looking for dominant angles?
  * **Add rectification error as an alternative**
    * As shown in [this paper](http://www.google.com/url?q=http%3A%2F%2Fzurich.disneyresearch.com%2Fderekbradley%2FPapers%2Fcrv10_CamCalib.pdf&sa=D&sntz=1&usg=AFQjCNFKp1OESIih3AaPcYgukHntpOIrqQ), rectification error can be used instead of reprojection error for extrinsic calibration, and it may be more appropriate for many applications. So it should be added as an extra mode.