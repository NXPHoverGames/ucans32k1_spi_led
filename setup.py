from setuptools import setup

package_name = 'ucans32k1_spi_led'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='Benjamin Perseghetti',
    author_email='bperseghetti@rudislabs.com',
    maintainer='Benjamin Perseghetti',
    maintainer_email='bperseghetti@rudislabs.com',
    description='UCANS32K1 SPI LED to Cyphal publisher node.',
    license='BSD-3',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ucans32k1_spi_led_node = ucans32k1_spi_led.ucans32k1_spi_led_node:main'
        ],
    },
)
