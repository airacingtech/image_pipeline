#!/usr/bin/env python3
"""Start ART monocular calibration with the approved camera-model contract."""

from __future__ import annotations

import argparse
import shlex
import sys


FISHEYE_CAMERAS = frozenset({
    'vimba_front',
    'vimba_left',
    'vimba_right',
    'vimba_rear',
})

PINHOLE_STEREO_CAMERAS = frozenset({
    'vimba_front_left_center',
    'vimba_front_right_center',
})

ART_CAMERAS = FISHEYE_CAMERAS | PINHOLE_STEREO_CAMERAS


def camera_model(camera_name: str) -> str:
    if camera_name in FISHEYE_CAMERAS:
        return 'fisheye'
    if camera_name in PINHOLE_STEREO_CAMERAS:
        return 'pinhole'
    raise ValueError(f'Unsupported ART camera: {camera_name}')


def build_cameracalibrator_args(
    camera_name: str,
    image_topic: str | None = None,
    auto_save: str | None = None,
    auto_progress: str | None = None,
    auto_exit: bool = False,
    headless: bool = False,
) -> list[str]:
    model = camera_model(camera_name)
    topic = image_topic or f'/{camera_name}/image'
    if not topic.startswith('/'):
        raise ValueError('--image-topic must be an absolute ROS topic')

    arguments = ['--camera-model', model]
    # ART automatic collection must be driven by complete pose coverage, not
    # the upstream fallback that declares any 40 accepted samples sufficient.
    arguments.append('--require-full-coverage')
    if model == 'fisheye':
        arguments.extend([
            '--fisheye-recompute-extrinsics',
            '--fisheye-check-conditions',
            '--fisheye-fix-skew',
        ])

    if auto_save:
        arguments.extend(['--auto-save', auto_save])
    if auto_progress:
        arguments.extend(['--auto-progress', auto_progress])
    if auto_exit:
        arguments.append('--auto-exit')
    if headless:
        arguments.append('--headless')

    arguments.extend([
        '--expected-width', '2064',
        '--expected-height', '1544',
        '--camera_name', camera_name,
        '--size', '7x10',
        '--square', '0.0700',
        '--no-service-check',
        # The rounded outer cells on the ART board can trigger false negatives
        # in OpenCV's inexpensive pre-check after the 2K image is downsampled.
        '--disable_calib_cb_fast_check',
        '--ros-args',
        '-r', f'image:={topic}',
    ])
    return arguments


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description='Run ART 2K monocular calibration with the fixed lens model.')
    parser.add_argument('camera_name', choices=sorted(ART_CAMERAS))
    parser.add_argument(
        '--image-topic',
        help='Raw image topic; defaults to /<camera_name>/image.',
    )
    parser.add_argument(
        '--print-command',
        action='store_true',
        help='Print the resolved cameracalibrator command without running it.',
    )
    parser.add_argument(
        '--auto-save', metavar='ARCHIVE',
        help='Automatically solve and save when pose coverage is sufficient.',
    )
    parser.add_argument(
        '--auto-progress', metavar='JSON',
        help='Write sample count and automatic calibration state to JSON.',
    )
    parser.add_argument(
        '--auto-exit', action='store_true',
        help='Exit after the automatic archive is saved.',
    )
    parser.add_argument(
        '--headless', action='store_true',
        help='Run without an OpenCV window; requires --auto-save.',
    )
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    if (args.auto_progress or args.auto_exit or args.headless) and not args.auto_save:
        raise SystemExit(
            '--auto-progress, --auto-exit, and --headless require --auto-save')
    try:
        calibrator_args = build_cameracalibrator_args(
            args.camera_name,
            args.image_topic,
            auto_save=args.auto_save,
            auto_progress=args.auto_progress,
            auto_exit=args.auto_exit,
            headless=args.headless,
        )
    except ValueError as exc:
        raise SystemExit(str(exc)) from exc

    command = [
        'ros2', 'run', 'camera_calibration', 'cameracalibrator',
        *calibrator_args,
    ]
    if args.print_command:
        print(shlex.join(command))
        return 0

    if args.camera_name in PINHOLE_STEREO_CAMERAS:
        print(
            'NOTE: this is only a pinhole monocular run. Use the ART center '
            'stereo SOP for the deployable joint stereo result.',
            file=sys.stderr,
        )

    from camera_calibration.nodes import cameracalibrator

    original_argv = sys.argv
    try:
        sys.argv = ['cameracalibrator', *calibrator_args]
        cameracalibrator.main()
    finally:
        sys.argv = original_argv
    return 0


if __name__ == '__main__':
    sys.exit(main())
