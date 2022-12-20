from setuptools import setup

package_name = "franka_avoidance"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        # ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="ros2",
    maintainer_email="ros2@todo.todo",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    # entry_points={
    #     "console_scripts": ["franka_node = franka_avoidance.franka_node:main"],
    # },
)
