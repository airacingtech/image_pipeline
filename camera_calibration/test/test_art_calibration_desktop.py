#!/usr/bin/env python3
"""Verify the five independent vehicle-side desktop calibration launchers."""

from pathlib import Path
import subprocess


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
SCRIPTS = {
    'vimba_front': ('art_calibrate_vimba_front.sh', '/vimba_front/image'),
    'vimba_left': ('art_calibrate_vimba_left.sh', '/vimba_left/image'),
    'vimba_right': ('art_calibrate_vimba_right.sh', '/vimba_right/image'),
    'vimba_rear': ('art_calibrate_vimba_rear.sh', '/vimba_rear/image'),
}
STEREO_SCRIPT = 'art_calibrate_center_stereo.sh'


def test_four_fisheye_scripts_are_independent_and_syntax_valid():
    for camera, (script_name, topic) in SCRIPTS.items():
        script = PACKAGE_ROOT / 'scripts' / script_name
        text = script.read_text(encoding='utf-8')

        subprocess.run(['bash', '-n', str(script)], check=True)
        assert f'CAMERA="{camera}"' in text
        assert f'TOPIC="{topic}"' in text
        assert 'art_camera_calibrator "${CAMERA}"' in text
        assert '--auto-save' in text
        assert '--auto-progress' in text
        assert '--auto-exit' in text
        assert 'test -s "${SESSION}/calibrationdata.tar.gz"' in text
        assert 'ros2 launch' not in text


def test_stereo_script_is_independent_and_syntax_valid():
    script = PACKAGE_ROOT / 'scripts' / STEREO_SCRIPT
    text = script.read_text(encoding='utf-8')

    subprocess.run(['bash', '-n', str(script)], check=True)
    assert 'ART_STEREO_LEFT_TOPIC:-/vimba_front_left_center/image' in text
    assert 'ART_STEREO_RIGHT_TOPIC:-/vimba_front_right_center/image' in text
    assert 'art_stereo_auto' in text
    assert '--max-pairs 0' in text
    assert '--expected-width 2064' in text
    assert '--expected-height 1544' in text
    assert 'ros2 launch' not in text


def test_all_scripts_source_vehicle_workspaces_and_do_not_start_cameras():
    scripts = list((PACKAGE_ROOT / 'scripts').glob('art_calibrate_*.sh'))
    assert len(scripts) == 5

    for script in scripts:
        text = script.read_text(encoding='utf-8')
        assert text.index('set +u') < text.index('source /opt/ros/jazzy/setup.bash')
        assert text.index('set -u', text.index('set +u')) > text.index(
            'source /home/autera-admin/ART/image_pipeline/install/setup.bash')
        assert 'source /opt/ros/jazzy/setup.bash' in text
        assert 'source /home/autera-admin/ART/race_common/install/setup.bash' in text
        assert 'source /home/autera-admin/ART/image_pipeline/install/setup.bash' in text
        assert 'ros2 topic info' in text
        assert 'vimba.launch.py' not in text


def test_five_desktop_entries_run_the_five_scripts_in_terminals():
    desktop_entries = list((PACKAGE_ROOT / 'desktop').glob('*.desktop'))
    assert len(desktop_entries) == 5

    expected_scripts = {
        script_name for script_name, _ in SCRIPTS.values()
    } | {STEREO_SCRIPT}
    actual_scripts = set()
    for desktop_entry in desktop_entries:
        text = desktop_entry.read_text(encoding='utf-8')
        assert 'Type=Application' in text
        assert 'Terminal=true' in text
        assert 'Exec=/home/autera-admin/ART/image_pipeline/' in text
        actual_scripts.add(text.split('Exec=', 1)[1].splitlines()[0].rsplit('/', 1)[1])

    assert actual_scripts == expected_scripts
