from setuptools import find_packages, setup

package_name = 'auspex_knowledge'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name, package_name+"/external_knowledge"],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Kai Sommer',
    maintainer_email='kai.sommer@unibw.de',
    description='AUSPEX World Knowledge Base',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'drone_state_mock_publisher = auspex_knowledge.drone_state_mock_publisher:main',
            'ul_state_mock_publisher = auspex_knowledge.ul_state_mock_publisher:main',
            'search_mission_mock_publisher = auspex_knowledge.search_mission_mock_publisher:main',
            'knowledge_main = auspex_knowledge.knowledge_main:main',
            'copernicus_server = auspex_knowledge.external_knowledge.copernicus_server:main',
        ],
    },
)
