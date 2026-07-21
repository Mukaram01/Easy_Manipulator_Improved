import { THREE } from './dist/viewer.bundle.js';

const BASELINE = Object.freeze({
  background: 0xf2f4f6,
  sky: 0xffffff,
  ground: 0xcbd3dc,
  exposure: 1.08,
});
const PATCH_FLAG = Symbol.for('workcell-studio.rviz-light-baseline.v1');
const configuredMeshes = new WeakSet();

function isHelper(node) {
  const data = node?.userData || {};
  const identity = [
    node?.name,
    data?.source_layer,
    data?.role,
    data?.category,
    data?.item?.source_layer,
    data?.item?.role,
    data?.item?.category,
  ].map(value => String(value || '').toLowerCase().replace(/[_-]+/g, ' ')).join(' ');
  return Boolean(
    node?.isGridHelper ||
    node?.isAxesHelper ||
    data?.exclude_from_fit_bounds === true ||
    /\b(debug|diagnostic|overlay|helper|zone|reach|fov|warning)\b/.test(identity)
  );
}

function isTranslucent(material) {
  const materials = Array.isArray(material) ? material : [material];
  return materials.some(entry => entry?.transparent && Number(entry.opacity ?? 1) < 0.9);
}

function configureScene(renderer, scene) {
  if (!renderer || !scene?.isScene) return;

  renderer.outputColorSpace = THREE.SRGBColorSpace;
  renderer.toneMapping = THREE.ACESFilmicToneMapping;
  renderer.toneMappingExposure = BASELINE.exposure;
  renderer.shadowMap.enabled = true;
  renderer.shadowMap.type = THREE.PCFSoftShadowMap;
  renderer.setClearColor(BASELINE.background, 1);

  if (!scene.background?.isColor) scene.background = new THREE.Color();
  scene.background.setHex(BASELINE.background);
  scene.fog = null;

  for (const child of scene.children) {
    if (child?.isHemisphereLight) {
      child.name = 'rviz_light_hemisphere';
      child.color.setHex(BASELINE.sky);
      child.groundColor.setHex(BASELINE.ground);
      child.intensity = 1.45;
    } else if (child?.isDirectionalLight) {
      child.name = 'rviz_light_key';
      child.color.setHex(0xffffff);
      child.intensity = 1.6;
      child.position.set(3.5, -4.5, 6.5);
      child.castShadow = true;
      child.shadow.mapSize.set(2048, 2048);
      Object.assign(child.shadow.camera, {
        near: 0.1,
        far: 20,
        left: -5,
        right: 5,
        top: 5,
        bottom: -5,
      });
      child.shadow.bias = -0.00025;
      child.shadow.normalBias = 0.018;
    } else if (child?.isGridHelper) {
      child.position.z = 0.002;
      child.renderOrder = -5;
      const materials = Array.isArray(child.material) ? child.material : [child.material];
      for (const material of materials) {
        if (!material) continue;
        material.transparent = true;
        material.opacity = 0.72;
        material.depthWrite = false;
      }
    }
  }

  scene.traverse(node => {
    if (!node?.isMesh || configuredMeshes.has(node)) return;
    configuredMeshes.add(node);
    const shadows = !isHelper(node) && !isTranslucent(node.material);
    node.castShadow = shadows;
    node.receiveShadow = shadows;
  });
}

function install() {
  const prototype = THREE.WebGLRenderer?.prototype;
  if (!prototype || prototype[PATCH_FLAG]) return;
  const originalRender = prototype.render;
  prototype.render = function renderWithRvizLightBaseline(scene, camera) {
    configureScene(this, scene);
    return originalRender.call(this, scene, camera);
  };
  prototype[PATCH_FLAG] = true;
  window.__WORKCELL_RVIZ_LIGHT_BASELINE__ = Object.freeze({
    enabled: true,
    version: 1,
    background: '#f2f4f6',
    shadows: 'pcf_soft',
    tone_mapping: 'aces_filmic',
  });
}

install();
