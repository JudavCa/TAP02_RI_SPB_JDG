# Taller — Cinemática y Planeación de Movimiento con ROS2 / MoveIt2 (UR3)

**Universidad EIA — Robótica Industrial — Parcial No. 2**

**Equipo:** _ET08_
**Integrantes:** _(Sebastian Palacio Barrientos, Juan David Guerra)_

---

## Requisitos

- Ubuntu 24.04 (Noble)
- ROS 2 Jazzy Jalisco
- MoveIt 2

```bash
sudo apt update
sudo apt install ros-jazzy-desktop ros-jazzy-moveit ros-jazzy-python-orocos-kdl-vendor
pip3 install numpy
```

---

## Compilación

```bash
git clone <URL_DEL_REPOSITORIO> ros2_2602
cd ros2_2602
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

> Después de cada `colcon build` hay que volver a ejecutar `source install/setup.bash`,
> y hacerlo en cada terminal nueva.

Para recompilar solo el paquete de los scripts:

```bash
colcon build --packages-select taller_pickplace
source install/setup.bash
```

---

## Ejecución

### 1. Levantar MoveIt2 y RViz2

En una **primera terminal** (dejarla corriendo):

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_2602/install/setup.bash
ros2 launch ur3_moveit_config demo.launch.py
```

### 2. Ejecutar los nodos

En una **segunda terminal**:

```bash
source ~/ros2_2602/install/setup.bash
```

| Comando | Descripción |
|---------|-------------|
| `ros2 run taller_pickplace ciclo_completo` | Ciclo completo de pick-and-place (4A → 4B → 4C → 4D) |
| `ros2 run taller_pickplace tramo_4b` | Tramo 4B (pre-pick → pick): compara perfiles cúbico y quíntico |
| `ros2 run taller_pickplace tramo_4d` | Tramo 4D (pre-place → place) |
| `ros2 run taller_pickplace jacobiano_moveit` | Imprime el Jacobiano de MoveIt en formato MATLAB |
| `ros2 run taller_pickplace velocidades_moveit` | Imprime `q` y `qdot` de los tramos 4B y 4D en formato MATLAB |

---

## MATLAB

El script `matlab/cinematica_ur3.m` contiene el modelo DH y el Jacobiano analítico.

Para usarlo:

1. Modificar el vector `q` con la configuración articular a evaluar
   (**vector columna**, separado por `;`).
2. Ejecutar para obtener la matriz homogénea `T` y el Jacobiano `J`.
3. Para verificar velocidades, sustituir `q_dot` por los valores que imprime
   `ros2 run taller_pickplace velocidades_moveit`.

---

## Notas

- El launch de MoveIt2 debe estar corriendo antes de ejecutar cualquier nodo.
- `ciclo_completo` asume que el robot parte de HOME.
- Si un tramo cartesiano reporta `fracción < 1.0`, verificar la pose de arranque
  y que no haya colisiones en la escena.
