from setuptools import find_packages, setup


package_name = "multi_uav_tf_manager"


setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        (
            "share/" + package_name,
            ["package.xml"],
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="liumengyang",
    maintainer_email="liumengyang@example.com",
    description="Static TF manager for the multi-UAV consensus stack.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "tf_manager = multi_uav_tf_manager.tf_manager:main",
        ],
    },
)
