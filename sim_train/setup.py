"""
Setup configuration for SO Arm Simulator package
"""

from setuptools import setup, find_packages

with open("README.md", "r", encoding="utf-8") as fh:
    long_description = fh.read()

setup(
    name="so-arm-simulator",
    version="0.1.0",
    author="Robotics Team",
    description="MuJoCo simulator for SO robotic arm with DualSense controller support",
    long_description=long_description,
    long_description_content_type="text/markdown",
    packages=find_packages(where="src"),
    package_dir={"": "src"},
    classifiers=[
        "Programming Language :: Python :: 3",
        "Programming Language :: Python :: 3.8",
        "Programming Language :: Python :: 3.9",
        "Programming Language :: Python :: 3.10",
        "Programming Language :: Python :: 3.11",
        "License :: OSI Approved :: MIT License",
        "Operating System :: OS Independent",
        "Topic :: Scientific/Engineering :: Robotics",
        "Development Status :: 3 - Alpha",
    ],
    python_requires=">=3.8",
    install_requires=[
        "mujoco>=3.0.0",
        "numpy>=1.20.0",
        "pygame>=2.1.0",
        "h5py>=3.0.0",
        "pyyaml>=6.0",
        "opencv-python>=4.5.0",
    ],
    extras_require={
        "dev": [
            "pytest>=7.0",
            "pytest-cov>=3.0",
            "black>=22.0",
            "isort>=5.10",
            "mypy>=0.950",
        ],
        "ml": [
            "torch>=1.10.0",
            "tensorflow>=2.8.0",
            "jax>=0.3.0",
        ],
    },
    entry_points={
        "console_scripts": [
            "so-arm-simulator=simulator:main",
            "so-motion-utils=motion_utils:main",
            "so-playback=playback:main",
        ],
    },
)
