from glob import glob

from setuptools import setup

package_name = "nav_dashboard"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/web", glob("web/*")),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="qtpi",
    maintainer_email="qtpi@example.com",
    description="Web dashboard for Nav2",
    license="MIT",
    entry_points={"console_scripts": [
        "dashboard = nav_dashboard.dashboard_node:main",
        "fake_nav = nav_dashboard.fake_nav:main",
    ]},
)
