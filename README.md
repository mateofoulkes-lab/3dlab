# 3DLab — Mesh to CAD

PWA experimental para reconstruir scans 3D de piezas manufacturadas como geometría CAD-like directamente en el dispositivo, sin subir el modelo a ningún servidor.

## v3.0

- Carga local de GLB, GLTF, STL y OBJ.
- Visor 3D táctil con Three.js.
- Detección de planos estructurales por conectividad, normales y error geométrico.
- Familias de caras paralelas.
- Snap ortogonal de familias cercanas a 90°.
- Reconstrucción de bordes como intersecciones rectas entre planos.
- Protección de zonas ambiguas para no forzar detalles no comprendidos.
- Detección experimental de separaciones repetidas entre planos paralelos como candidatos a espesores nominales.
- Comparación Scan / Reconstruido.
- Exportación local a STL u OBJ.
- Procesamiento en Web Worker.
- PWA instalable, offline y con actualización automática al refrescar cuando hay una versión nueva.

## Filosofía

El objetivo ya no es suavizar un mesh. Es inferir la intención de diseño de una pieza física escaneada: reemplazar ruido por planos, paralelismos, perpendicularidades, aristas rectas y, progresivamente, radios/fillets, cilindros, agujeros, simetrías y espesores consistentes.

## Próxima etapa

El siguiente gran paso es usar las caras reconstruidas como esqueleto geométrico para detectar las bandas curvas existentes entre planos y convertirlas en fillets de radio constante. Después: cilindros/agujeros y salida CAD paramétrica.

## GitHub Pages / Android

Publicar desde **Settings → Pages → Deploy from a branch → main / root**.

`https://mateofoulkes-lab.github.io/3dlab/`

En Android se puede instalar como PWA desde Chrome. Los modelos seleccionados permanecen en el dispositivo.
