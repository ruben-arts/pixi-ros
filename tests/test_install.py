"""Regression coverage for installing without a separate initialization step."""

from types import SimpleNamespace

import pytest
import tomlkit
from rattler import Platform
from typer.testing import CliRunner

from pixi_ros.cli import app
from pixi_ros.validator import RosDistroValidator

runner = CliRunner()


@pytest.fixture
def workspace(tmp_path, monkeypatch):
    package = tmp_path / "src" / "robot"
    package.mkdir(parents=True)
    (package / "package.xml").write_text(
        '<package format="3"><name>robot</name><version>0.1.0</version>'
        "<description>Robot</description>"
        '<maintainer email="test@example.com">Test</maintainer>'
        "<license>MIT</license><depend>rclpy</depend><depend>cmake</depend></package>"
    )
    monkeypatch.chdir(tmp_path)
    monkeypatch.delenv("ROS_DISTRO", raising=False)
    monkeypatch.setattr("pixi_ros.install.shutil.which", lambda _: "/pixi")
    monkeypatch.setattr(
        "pixi_ros.install.subprocess.run",
        lambda *a, **kw: SimpleNamespace(returncode=0),
    )
    monkeypatch.setattr(
        "pixi_ros.validator._DISTRO_CACHE",
        {"jazzy": SimpleNamespace(release_packages={"rclpy", "ros2cli"})},
    )
    monkeypatch.setattr(
        RosDistroValidator,
        "check_package_availability",
        lambda self, name, platform, channel: name.startswith("ros-jazzy-"),
    )
    return tmp_path


def test_install_without_manifest_prompts_and_creates_native_workspace(workspace):
    result = runner.invoke(app, ["install"], input="invalid\njazzy\n")
    assert result.exit_code == 0, result.output
    config = tomlkit.loads((workspace / "pixi.toml").read_text())
    assert "ros-jazzy-rclpy" in config["dependencies"]
    assert "ros-jazzy-robot" not in config["dependencies"]
    assert config["tasks"]["build"]["cmd"] == "colcon build"
    assert not (workspace / "README_PIXI.md").exists()
    for script in config.get("activation", {}).get("scripts", []):
        assert (workspace / script).exists()


def test_install_reuses_manifest_without_prompt_and_preserves_user_settings(workspace):
    manifest = workspace / "pixi.toml"
    manifest.write_text(
        '# Keep this comment\n[workspace]\nname = "my-robot"\n'
        'channels = ["https://prefix.dev/robostack-jazzy"]\n'
        f'platforms = ["{Platform.current()}"]\n'
        '[dependencies]\ncmake = "<4"\n'
        'ros-jazzy-rclpy = { version = "*", channel = "robostack-jazzy" }\n'
        '[tasks]\nbuild = "colcon build --symlink-install"\n'
        "[activation]\nscripts = []\n"
    )
    result = runner.invoke(app, ["install"])
    assert result.exit_code == 0, result.output
    config = tomlkit.loads(manifest.read_text())
    assert config["dependencies"]["cmake"] == "<4"
    assert config["dependencies"]["ros-jazzy-rclpy"]["channel"] == "robostack-jazzy"
    assert config["tasks"]["build"] == "colcon build --symlink-install"
    assert config["activation"]["scripts"] == []
    assert "# Keep this comment" in manifest.read_text()
    first = manifest.read_text()
    again = runner.invoke(app, ["install"])
    assert again.exit_code == 0, again.output
    assert manifest.read_text() == first


def test_install_uses_workspace_local_mapping_from_nested_package(
    workspace, monkeypatch
):
    mappings = workspace / "pixi-ros"
    mappings.mkdir()
    (mappings / "custom.yaml").write_text("rclpy:\n  conda-forge: [robot-python]\n")
    monkeypatch.chdir(workspace / "src" / "robot")
    result = runner.invoke(app, ["install", "-d", "jazzy"])
    assert result.exit_code == 0, result.output
    config = tomlkit.loads((workspace / "pixi.toml").read_text())
    assert "robot-python" in config["dependencies"]
    assert "ros-jazzy-rclpy" not in config["dependencies"]
    assert not (workspace / "src" / "robot" / "pixi.toml").exists()


def test_install_keeps_resolved_custom_channel_name(workspace, monkeypatch):
    package = workspace / "src" / "robot" / "package.xml"
    package.write_text(package.read_text().replace("rclpy", "custom_driver"))
    monkeypatch.setattr(
        RosDistroValidator,
        "check_package_availability",
        lambda self, name, platform, channel: (
            name.startswith("ros-jazzy-ros2cli")
            or (name == "custom-driver" and channel == "https://example.org/robotics")
        ),
    )
    result = runner.invoke(
        app, ["install", "-d", "jazzy", "-c", "https://example.org/robotics"]
    )
    assert result.exit_code == 0, result.output
    config = tomlkit.loads((workspace / "pixi.toml").read_text())
    assert "custom-driver" in config["dependencies"]
    assert "ros-jazzy-custom-driver" not in config["dependencies"]


def test_unresolved_dependency_does_not_modify_manifest(workspace, monkeypatch):
    manifest = workspace / "pixi.toml"
    original = (
        '[workspace]\nchannels = ["robostack-jazzy"]\n'
        f'platforms = ["{Platform.current()}"]\n'
    )
    manifest.write_text(original)
    monkeypatch.setattr(
        RosDistroValidator, "check_package_availability", lambda *a, **kw: False
    )
    result = runner.invoke(app, ["install"])
    assert result.exit_code == 1, result.output
    assert "rclpy" in result.output
    assert manifest.read_text() == original


def test_install_propagates_pixi_failure(workspace, monkeypatch):
    monkeypatch.setattr(
        "pixi_ros.install.subprocess.run",
        lambda *a, **kw: SimpleNamespace(returncode=17),
    )
    result = runner.invoke(app, ["install", "-d", "jazzy"])
    assert result.exit_code == 17, result.output
    assert "Environment installed" not in result.output


def test_conflicting_distro_does_not_modify_manifest(workspace):
    manifest = workspace / "pixi.toml"
    original = '[workspace]\nchannels = ["robostack-humble"]\n'
    manifest.write_text(original)
    result = runner.invoke(app, ["install", "-d", "jazzy"])
    assert result.exit_code != 0
    assert manifest.read_text() == original


def test_missing_pixi_does_not_create_manifest(workspace, monkeypatch):
    monkeypatch.setattr("pixi_ros.install.shutil.which", lambda _: None)
    result = runner.invoke(app, ["install", "-d", "jazzy"])
    assert result.exit_code == 1, result.output
    assert not (workspace / "pixi.toml").exists()


def test_install_cancelled_before_platform_change(workspace):
    manifest = workspace / "pixi.toml"
    foreign = "linux-64" if str(Platform.current()) != "linux-64" else "osx-arm64"
    original = (
        f'[workspace]\nchannels = ["robostack-jazzy"]\nplatforms = ["{foreign}"]\n'
    )
    manifest.write_text(original)
    result = runner.invoke(app, ["install"], input="n\n")
    assert result.exit_code != 0
    assert manifest.read_text() == original
