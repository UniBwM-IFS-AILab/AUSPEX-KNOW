from setuptools import find_packages, setup

package_name = 'auspex_knowledge'

setup(
    name=package_name,
    version='2.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Kai Sommer',
    maintainer_email='kai.sommer@unibw.de',
    description='AUSPEX Knowledge Base',
    license='MIT',
    entry_points={
        'console_scripts': [
            'knowledge_main = auspex_knowledge.knowledge_main:main',
        ],
    },
)
