"""test_arm_sdk install entry point — `pip3 install -e .` from this directory."""
from setuptools import setup, find_packages

setup(
    name='test_arm_sdk',
    version='1.0.0',
    description='Virtual 6-DOF arm SDK for EasyTrainer wizard testing.',
    author='EasyTrainer test fixture',
    packages=find_packages(),
    install_requires=[],
    python_requires='>=3.8',
)
