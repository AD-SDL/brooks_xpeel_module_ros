from setuptools import setup, find_packages


install_requires = ['pyserial']

package_name = 'brooks_peeler_driver'

setup(
    name='brooks_peeler_driver',
    version='0.0.1',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=install_requires,
    zip_safe=True,
    python_requires=">=3.8",
    maintainer='Rafael Vescovi and Doga Ozgulbas',
    maintainer_email='dozgulbas@anl.gov',
    description='Driver for the Azenta Sealer and Peeler',
    url='https://github.com/AD-SDL/brooks_xpeel_module.git', 
    license='MIT License',
    entry_points={},
    classifiers=[
        'Intended Audience :: Science/Research',
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Operating System :: POSIX',
        'Operating System :: MacOS :: MacOS X',
        'Programming Language :: Python',
        'Programming Language :: Python :: 3.8',
        'Programming Language :: Python :: 3.9',
    ],
)
