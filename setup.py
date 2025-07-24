from setuptools import setup, find_packages
import os

def read_readme():
    path = os.path.join(os.path.dirname(__file__), 'README.md')
    return open(path, 'r', encoding='utf-8').read() if os.path.exists(path) else ""

def read_requirements():
    path = os.path.join(os.path.dirname(__file__), 'requirements.txt')
    return [line.strip() for line in open(path, 'r', encoding='utf-8') if line.strip() and not line.startswith('#')]

def get_version():
    path = os.path.join(os.path.dirname(__file__), 'bessica_d_sdk', '__init__.py')
    for line in open(path, 'r', encoding='utf-8'):
        if line.startswith('__version__'):
            return line.split('=')[-1].strip().strip('"').strip("'")
    return "0.1.0"

setup(
    name='Bessica-D-SDk',
    version=get_version(),
    author='Xuanya Robotics',
    author_email='tech@xuanyatech.com',
    description='Python SDK for controlling the Bessica-D dual-arm 7-DoF robotic platform',
    long_description=read_readme(),
    long_description_content_type='text/markdown',
    url='https://github.com/Xuanya-Robotics/Bessica-D-SDK',
    packages=find_packages(exclude=['tests*', 'examples*']),
    install_requires=read_requirements(),
    classifiers=[
        'Development Status :: 4 - Beta',
        'Intended Audience :: Developers',
        'Intended Audience :: Science/Research',
        'License :: OSI Approved :: MIT License',
        'Programming Language :: Python :: 3',
        'Operating System :: OS Independent',
    ],
    python_requires='>=3.6',
    keywords='robotic arm, Bessica-D, dual-arm, SDK, robotics',
)
