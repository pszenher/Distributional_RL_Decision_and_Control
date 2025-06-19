from setuptools import setup

PACKAGE_NAME = 'rfarl'

setup(
    name=PACKAGE_NAME,
    version='0.1.0',
    packages=[],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + PACKAGE_NAME]),
        ('share/' + PACKAGE_NAME, ['package.xml']),
    ],
    install_requires=[
        'setuptools',
    ],
    author='Xi Lin',
    author_email='xlin26@stevens.edu',
    maintainer='Xi Lin',
    maintainer_email='xlin26@stevens.edu',
    tags=[
        'reinforcement-learning',
    ],
    classifiers=[
        'License :: OSI Approved :: MIT License'
        'Topic :: Scientific/Engineering :: Artificial Intelligence'
    ],
    license='MIT',
    entry_points={},
)
