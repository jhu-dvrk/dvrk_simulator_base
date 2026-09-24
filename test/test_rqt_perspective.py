import ast
import json
import os

from dvrk_simulator_base.rqt_perspective import (
    existing_ament_prefix_path,
    write_monitor_perspective,
)


def test_monitor_perspective_contains_the_tabbed_arms_plugin(tmp_path):
    path = write_monitor_perspective(
        tmp_path / "monitor.perspective", ("PSM1", "ECM", "PSM1"),
        include_console=True,
    )

    document = json.loads(path.read_text(encoding="utf-8"))
    plugins = ast.literal_eval(
        document["groups"]["pluginmanager"]["keys"]["running-plugins"]["repr"]
    )
    assert plugins == {
        "rqt_dvrk/Arms": [1],
        "rqt_dvrk/Diagnostics": [1],
        "rqt_dvrk/Console": [1],
    }


def test_existing_ament_prefix_path_drops_removed_prefixes(monkeypatch, tmp_path):
    deleted = tmp_path / "deleted"
    monkeypatch.setenv("AMENT_PREFIX_PATH", os.pathsep.join((str(tmp_path), str(deleted))))

    assert existing_ament_prefix_path() == str(tmp_path)
