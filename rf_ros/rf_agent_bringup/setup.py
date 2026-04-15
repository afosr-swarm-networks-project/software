from setuptools import setup

package_name = "rf_agent_bringup"

setup(
    name=package_name,
    version="0.0.1",
    packages=[],
    data_files=[
        ("share/ament_index/resource_index/packages",
         [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", ["launch/pipeline.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Zhongzheng Zhang",
    maintainer_email="zhangrenzhongzheng@outlook.com",
    description="Launch files for RF agent processing pipelines",
    license="Apache-2.0",
)
