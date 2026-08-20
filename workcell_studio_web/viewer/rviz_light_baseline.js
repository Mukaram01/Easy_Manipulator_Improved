import * as THREE from 'three';

const BASELINE = Object.freeze({
  background: 0xeef1f4,
  sky: 0xffffff,
  groundBounce: 0xcbd3dc,
  gridMajor: 0x8996a3,
  gridMinor: 0xb6c0ca,
  exposure: 1.08,
});
const PATCH_FLAG = Symbol.for('workcell-studio.rviz-light-baseline.v1');
const configuredRenderers = new WeakSet();
const configuredScenes = new WeakSet();
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

function configureRenderer(renderer) {
  if (configuredRenderers.has(renderer)) return;
  configuredRenderers.add(renderer);
  renderer.outputColorSpace = THREE.SRGBColorSpace;
  renderer.toneMapping = THREE.ACESFilmicToneMapping;
  renderer.toneMappingExposure = BASELINE.exposure;
  renderer.shadowMap.enabled = true;
  renderer.shadowMap.type = THREE.PCFSoftShadowMap;
  renderer.setClearColor(BASELINE.background, 1);
}

function configureScene(scene) {
  if (configuredScenes.has(scene)) return;
  configuredScenes.add(scene);
  if (!scene.background?.isColor) scene.background = new THREE.Color();
  scene.background.setHex(BASELINE.background);
  scene.fog = null;

  for (const child of scene.children) {
    if (child?.isHemisphereLight) {
      child.name = 'rviz_light_hemisphere';
      child.color.setHex(BASELINE.sky);
      child.groundColor.setHex(BASELINE.groundBounce);
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
      child.shadow.camera.updateProjectionMatrix();
      child.shadow.bias = -0.00025;
      child.shadow.normalBias = 0.018;
    } else if (child?.isGridHelper) {
      child.position.z = 0.002;
      child.renderOrder = -5;
      const materials = Array.isArray(child.material) ? child.material : [child.material];
      materials.forEach((material, index) => {
        if (!material) return;
        material.color?.setHex(index === 0 ? BASELINE.gridMajor : BASELINE.gridMinor);
        material.transparent = true;
        material.opacity = index === 0 ? 0.82 : 0.62;
        material.depthWrite = false;
      });
    }
  }
}

function configureNewMeshShadows(scene) {
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
    configureRenderer(this);
    configureScene(scene);
    const frame = (scene.userData.rvizLightMeshScanFrame || 0) + 1;
    scene.userData.rvizLightMeshScanFrame = frame % 20;
    if (frame === 1 || frame % 20 === 0) configureNewMeshShadows(scene);
    return originalRender.call(this, scene, camera);
  };
  prototype[PATCH_FLAG] = true;
  window.__WORKCELL_RVIZ_LIGHT_BASELINE__ = Object.freeze({
    enabled: true,
    version: 1,
    background: '#eef1f4',
    shadows: 'pcf_soft',
    tone_mapping: 'aces_filmic',
  });
}

install();
