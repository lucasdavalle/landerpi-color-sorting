# landerpi_color_sorting

> **En desarrollo** — etapa 0 de 6 completa (detector de color). Demo en
> video cuando esté terminada la etapa 6.

Proyecto de portfolio (B3) sobre el robot **LanderPi** (base mecanum + brazo
de 6 GDL + cámara Aurora930): una estación de clasificación por color en C++
que reemplaza `color_sorting_node.py` del stack Python de fábrica. El
original mueve el brazo con secuencias pregrabadas y solo agarra un bloque si
está en un recuadro fijo de ~20 px de la imagen, sin verificar el resultado.
Esta versión detecta el bloque **en cualquier lugar de la zona de trabajo**,
calcula la pose de agarre con cinemática inversa, lo deja en el depósito de
su color, verifica con la cámara que el agarre y el depósito realmente
ocurrieron (reintentando si no) y lleva un marcador de aciertos por color.

Paquete:

- [`landerpi_color_sorting`](landerpi_color_sorting/README.md) — el nodo en
  sí (`ament_cmake`, C++).

## Contexto técnico

Desarrollado y probado en hardware real (LanderPi corriendo ROS 2 Humble
dentro de un contenedor Docker en una Raspberry Pi). Todas las integraciones
se verifican contra el estado real y en vivo del robot, no solo contra la
documentación o el código fuente del stock — por ejemplo,
`/controller_manager/servo_states` del stock **no es feedback real** de los
servos (republica la última posición comandada), por eso la verificación del
agarre se hace con la cámara.

## Licencia

MIT (ver [`LICENSE`](LICENSE)). El paquete ROS en sí declara Apache-2.0 en
su `package.xml`.
