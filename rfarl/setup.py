from setuptools import find_packages, setup

PACKAGE_NAME = 'rfarl'

setup(
    name=PACKAGE_NAME,
    version='0.1.1',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + PACKAGE_NAME]),
        ('share/' + PACKAGE_NAME, ['package.xml']),
    ],
    author='Xi Lin',
    author_email='xlin26@stevens.edu',
    maintainer='Xi Lin',
    maintainer_email='xlin26@stevens.edu',
    description='Robust Field Autonomy Reinforcement-Learning',
    keywords=[
        'reinforcement-learning',
    ],
    classifiers=[
        'License :: OSI Approved :: MIT License',
        'Topic :: Scientific/Engineering :: Artificial Intelligence',
    ],
    license='MIT',
    entry_points={
        'console_scripts': []
    },
)
