from setuptools import setup, find_packages
import os

def read_readme():
    readme_path = os.path.join(os.path.dirname(__file__), 'README.md')
    if os.path.exists(readme_path):
        with open(readme_path, 'r', encoding='utf-8') as f:
            return f.read()
    return "Python SDK for controlling the Bessica-D robotic arm."

def read_requirements():
    requirements_path = os.path.join(os.path.dirname(__file__), 'requirements.txt')
    if os.path.exists(requirements_path):
        with open(requirements_path, 'r', encoding='utf-8') as f:
            return [line.strip() for line in f if line.strip() and not line.startswith('#')]
    return []

def get_version():
    init_py_path = os.path.join(os.path.dirname(__file__), 'bessica_d_sdk', '__init__.py')
    version_line = ''
    if os.path.exists(init_py_path):
        with open(init_py_path, 'r', encoding='utf-8') as f:
            for line in f:
                if line.startswith('__version__'):
                    version_line = line
                    break

    if version_line:
        return version_line.split('=')[-1].strip().strip('"').strip("'")
    return "0.1.0" # 默认版本

setup(
    name='bessica_d_sdk',
    version=get_version(),
    author='Xuanya Robotics', 
    author_email='tech@xuanyatech.com', 
    description='Python SDK for controlling the Bessica-D robotic arm',
    long_description=read_readme(),
    long_description_content_type='text/markdown',
    url='https://github.com/Xuanya-Robotics/Bessica-D-SDK', 
    packages=find_packages(exclude=['tests*', 'examples*']), 
    install_requires=read_requirements(),
    classifiers=[
        'Development Status :: 3 - Alpha', 
        'Intended Audience :: Developers',
        'Intended Audience :: Education',
        'Intended Audience :: Science/Research',
        'License :: OSI Approved :: MIT License',  
        'Programming Language :: Python :: 3',
        'Programming Language :: Python :: 3.6',
        'Programming Language :: Python :: 3.7',
        'Programming Language :: Python :: 3.8',
        'Programming Language :: Python :: 3.9',
        'Programming Language :: Python :: 3.10',
        'Programming Language :: Python :: 3.11',
        'Operating System :: OS Independent',
        'Topic :: Software Development :: Libraries :: Python Modules',
        'Topic :: System :: Hardware :: Hardware Drivers',
    ],
    python_requires='>=3.6',
    keywords='robotic arm, Bessica-D, sdk, serial communication, robotics',
)