ART Center Stereo Calibration
=============================

This additive workflow calibrates the ART center stereo pair without changing
the existing ``cameracalibrator`` command or its configuration:

* left: ``vimba_front_left_center`` (``10.42.17.53``)
* right: ``vimba_front_right_center`` (``10.42.17.51``)
* target: 11 by 8 squares, therefore 10 by 7 OpenCV inner corners
* square size: 70 mm
* capture contract: 2064 by 1544 at 10 Hz, synchronized with Action0

The vehicle publishes the images. The capture process runs on the ``roar``
workstation and stores every raw pair locally. Corner detection and calibration
then run offline, so the operator does not need to hold the target while the
computer accepts or rejects a view.

Build and source
----------------

Build this branch in the normal vehicle and workstation workspaces, then source
the resulting setup file. On the vehicle, also source ``race_common`` so that
``iac_launch`` and ``avt_vimba_camera`` are available.

Vehicle capture launch
----------------------

Stop any process that already owns either center camera, and launch the
dedicated full-resolution calibration publishers:

.. code-block:: bash

   ros2 launch camera_calibration art_stereo_capture.launch.py

The launch reads the existing per-camera files from ``iac_launch`` and layers
the installed ``art_stereo_vimba_override.yaml`` on top. The original files are
not modified. Before collecting data, check the active topics for ten seconds:

.. code-block:: bash

   ros2 run camera_calibration art_stereo_status \
     --duration 10 \
     --json-out /tmp/art_stereo_preflight.json

The required preflight gates are two streams, matching 2064 by 1544 images,
10 Hz rates, and a timestamp-pairing p95 no greater than 2 ms. A zero baseline
in CameraInfo is expected before calibration unless ``--require-baseline`` is
requested.

Capture on the roar workstation
-------------------------------

Use the same ROS domain as the vehicle. If direct Cyclone DDS discovery is
needed, copy and edit the installed example for the workstation interface and
vehicle address before exporting it:

.. code-block:: bash

   export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
   export CYCLONEDDS_URI=file://$(ros2 pkg prefix camera_calibration)/share/camera_calibration/config/art_stereo_cyclonedds_roar_vehicle.xml

Create a new output directory for every session. The capture command refuses to
append to a non-empty session:

.. code-block:: bash

   SESSION=$HOME/stereo_calibration_data/$(date +%Y%m%d_%H%M%S)
   ros2 run camera_calibration art_stereo_capture \
     --output "$SESSION" \
     --left-topic /vimba_calib_left/image \
     --right-topic /vimba_calib_right/image \
     --mode interval \
     --max-pairs 60 \
     --max-delta-ms 2.0 \
     --min-interval-sec 1.0

Two people are recommended: one moves the board and calls each pose, while the
other watches the saved-pair count and image streams. Hold each pose for 1.5 to
2 seconds. Collect 50 to 60 raw pairs covering the center, four corners, four
edges, near and far distances, and positive and negative yaw, pitch, and roll.
Keep the whole checkerboard visible and avoid blur, glare, or repeated poses.

Offline calibration
-------------------

Measure the physical optical-center baseline when possible, then run:

.. code-block:: bash

   RESULT=${SESSION}_result
   ros2 run camera_calibration art_stereo_calibrate \
     --input "$SESSION" \
     --output "$RESULT" \
     --expected-width 2064 \
     --expected-height 1544 \
     --expected-baseline-m <measured-baseline-in-metres>

Omit ``--expected-baseline-m`` only when no reliable mechanical measurement is
available. The offline stage re-detects both boards, rejects unreadable and
high-error pairs, reserves a holdout subset, and writes:

* ``report.json`` and ``report.md`` with all acceptance gates
* ``stereo_calibration.yaml`` with the left-to-right transform and rectification
* ``left_camera_info.yaml`` and ``right_camera_info.yaml``
* rectified preview images for a final visual epipolar check

Completion criteria
-------------------

A session is complete only when ``report.json`` says ``PASS`` and the rectified
previews place corresponding target points on the same horizontal rows. The
default gates require at least 15 accepted views, stereo RMS at most 1.0 px,
final epipolar p95 at most 1.0 px, holdout epipolar p95 at most 1.5 px,
timestamp delta p95 at most 2 ms, a nonzero baseline, and a nonzero right-camera
projection translation. If a mechanical baseline was supplied, the calibrated
baseline must also be within 5 percent.

Do not overwrite production CameraInfo files automatically. Review the report,
preview images, image size, transform direction, and baseline before deploying
the two generated CameraInfo YAML files.
