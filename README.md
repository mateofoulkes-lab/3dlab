# 3DLab

MVP web/PWA para limpiar meshes 3D escaneados directamente en el dispositivo, sin subir archivos a un servidor.

## MVP actual

- Carga local de STL y OBJ.
- Visor 3D táctil con Three.js.
- Detección de regiones aproximadamente planas por conectividad y normales.
- Proyección gradual hacia planos ideales con control **Fuerza CAD**.
- Tolerancia angular y tamaño mínimo de región configurables.
- Comparación Original / Corregido.
- Exportación local a STL u OBJ.
- Procesamiento en Web Worker para no bloquear la interfaz.
- PWA instalable y cache offline después de la primera carga completa.

## Publicar con GitHub Pages

En el repositorio: **Settings → Pages → Deploy from a branch → main / root**.

La URL esperada es:

`https://mateofoulkes-lab.github.io/3dlab/`

En Android, abrir esa URL con Chrome y usar **Instalar app / Añadir a pantalla principal**. Los modelos elegidos desde el selector de archivos permanecen en el dispositivo.

## Alcance de esta primera prueba

Esta versión prueba la hipótesis principal: corregir el aspecto bumpy de piezas manufacturadas detectando caras que deberían ser planas. El siguiente paso previsto es agregar restricciones CAD entre superficies: paralelismo, perpendicularidad, espesores repetidos, líneas largas rectas y cilindros.
