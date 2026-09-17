from setuptools import find_packages, setup

package_name = 'taller_pickplace'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sebas',
    maintainer_email='sebas@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },

entry_points={
    'console_scripts': [
'tramo_4b = taller_pickplace.tramo_4b:main',
'tramo_4d = taller_pickplace.tramo_4d:main',
'ciclo_completo = taller_pickplace.ciclo_completo:main',
'jacobiano_moveit = taller_pickplace.jacobiano_moveit:main',
'velocidades_moveit = taller_pickplace.velocidades_moveit:main',
 'verificar_jacobiano = taller_pickplace.verificar_jacobiano:main',
    ],
},



)
