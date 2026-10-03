# landerpi_color_sorting

> **En desarrollo.** Este README se completa (arquitectura, topics,
> parámetros, cómo correrlo, demo) a medida que avanzan las etapas.

## Estado

| Etapa | Qué | Estado |
|---|---|---|
| 0 | `ColorBlockDetector`: detector de bloques de color (Lab) | hecho |
| 1 | Pose de observación del brazo (IK + `follow_joint_trajectory`) | — |
| 2 | Mapeo píxel → mesa (homografía) + calibración | — |
| 3 | Pick & place de un bloque | — |
| 4 | Máquina de estados + depósitos por color | — |
| 5 | Verificación por cámara + reintento + marcador | — |
| 6 | Demo 4 rondas × 3 bloques | — |

## `ColorBlockDetector`

Clase pura (OpenCV, sin `rclcpp`): recibe un frame BGR y devuelve todos los
bloques de cada color calibrado (`color`, centro en píxeles, área, ángulo,
bounding box), ordenados de mayor a menor área. Umbral en espacio Lab con
tolerancia de luminosidad más amplia que la de croma, para tolerar sombras.
Los bloques cortados por el borde de la imagen se reportan marcados
(`touches_image_border`), porque su centroide visible no es el centro real
del bloque.

Los colores se calibran muestreando la imagen en un punto
(`SampleLab(frame, x_frac, y_frac)`), igual que hará el flujo de calibración
por click.

## Tests

Tests con fotos reales de la cámara del robot en `test/data/` (bloques en
fila; desparramados, girados y en sombra; dos bloques pegados; un bloque
cortado por el borde), calibrando con la primera foto y detectando en las
demás.

```bash
colcon build --packages-select landerpi_color_sorting
colcon test --packages-select landerpi_color_sorting
colcon test-result --verbose
```
