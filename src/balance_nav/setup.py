from setuptools import find_packages, setup


package_name = "balance_nav"


setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", ["launch/balance_nav.launch.py"]),
        (f"share/{package_name}/config", ["config/nav_params.yaml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="lzy",
    maintainer_email="lzy@todo.todo",
    description="Minimal goal navigation and obstacle avoidance for the balance car workspace.",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "cmd_vel_gate_node = balance_nav.cmd_vel_gate_node:main",
            "odometry_node = balance_nav.odometry_node:main",
        ],
    },
)
