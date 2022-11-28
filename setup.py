from setuptools import setup, find_packages

package_name = "franka_avoidance"

setup(
    name=package_name,
    version="0.0.0",
    # packages=[package_name],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Lukas Huber",
    maintainer_email="lukas.huber@epfl.ch",
    description="Obstacle Avoidance with the Franka Robot",
    license="LICENSE",
    package_dir={"": "src"},
    packages=find_packages(where="src", include=[package_name]),
    tests_require=["pytest"],
    # entry_points={
    # 'console_scripts': ['simulation_loader = pybullet_ros2.simulation_loader:main',
    # 'pybullet_ros2 = pybullet_ros2.pybullet_ros2:main']
    # }
)
