import os
from glob import glob

from setuptools import find_packages, setup


package_name = 'nero_ctrl'

setup(
	name=package_name,
	version='0.0.1',
	packages=find_packages(exclude=['test']),
	data_files=[
		(
			'share/ament_index/resource_index/packages',
			['resource/' + package_name],
		),
		(
			'share/' + package_name,
			['package.xml'],
		),
		(
			os.path.join('share', package_name, 'launch'),
			glob('launch/*.launch.py'),
		),
		(
			os.path.join('share', package_name, 'models', 'escena_cinta_grupo'),
			glob('models/escena_cinta_grupo/*.xml'),
		),
		(
			os.path.join('share', package_name, 'models', 'escena_cinta_grupo', 'assets'),
			glob('models/escena_cinta_grupo/assets/*'),
		),
	],
	install_requires=['setuptools', 'python-can'],
	zip_safe=True,
	entry_points={
		'console_scripts': [
			# Nodos originales iPre_nero
			'can_module = nero_ctrl.can_module:main',
			'main_node = nero_ctrl.main_node:main',
			'controller_node = nero_ctrl.controller_node:main',
			'ik_node = nero_ctrl.IK_node:main',
			'calibration_node = nero_ctrl.calibration_module:main',
			# Puente MuJoCo empaquetado (común)
			'nero_sim_bridge = nero_ctrl.comun.simulador_mujoco_bridge:main',
			# Tarea 1 - Manejo del Robot
			'tarea1_1_gemelo = nero_ctrl.tarea1_manejo.gemelo_digital_mujoco:main',
			'tarea1_1_emulador = nero_ctrl.tarea1_manejo.emulador_estado_real:main',
			'tarea1_2_recoger_cinta = nero_ctrl.tarea1_manejo.recoger_cubo_cinta:main',
			'tarea1_3_piramide = nero_ctrl.tarea1_manejo.armar_piramide:main',
			# Tarea 2 - Generadores de referencia (se comunican con `controller_node`)
			'tarea2_1_estatica = nero_ctrl.tarea2_control.tarea2_1_estatica:main',
			'tarea2_2_movil = nero_ctrl.tarea2_control.tarea2_2_movil:main',
		],
	},
)
