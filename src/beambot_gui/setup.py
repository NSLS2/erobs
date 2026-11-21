from setuptools import setup

package_name = 'beambot_gui'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    package_data={package_name: ['resources/*']},
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/beambot_gui_client.launch.py']),
    ],
    install_requires=['setuptools'],
    maintainer='abondada',
    maintainer_email='abondada@bnl.gov',
    description='Graphical User Interface for beambot pipeline',
    license='BSD-3-Clause',
    entry_points={
        'console_scripts': [
            'beambot_gui_client = beambot_gui.main:main',
        ],
    },
)
