from setuptools import setup, find_packages

setup(
    name="anti-drone-sim",
    version="0.1.0",
    description="反无人机数据生成系统 - 多传感器目标轨迹仿真",
    packages=find_packages(where="src"),
    package_dir={"": "src"},
    python_requires=">=3.10",
    install_requires=[
        "numpy>=1.24.0",
        "scipy>=1.10.0",
        "pyqtgraph>=0.13.0",
        "PyQt6>=6.5.0",
        "PyOpenGL>=3.1.7",
        "pyzmq>=25.0.0",
        "pyyaml>=6.0",
        "pydantic>=2.0.0",
    ],
    entry_points={
        "console_scripts": [
            "anti-drone-sim=main:main",
        ],
    },
)
