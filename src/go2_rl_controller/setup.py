from glob import glob
from setuptools import setup

package_name = "go2_rl_controller"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/policy", glob("policy/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    entry_points={
        "console_scripts": [
            "command_mux = go2_rl_controller.command_mux:main",
            "onnx_policy_controller = go2_rl_controller.onnx_policy_controller:main",
            "keyboard_teleop = go2_rl_controller.keyboard_teleop:main",
        ],
    },
)
