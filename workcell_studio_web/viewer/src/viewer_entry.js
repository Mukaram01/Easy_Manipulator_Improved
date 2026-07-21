import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { TransformControls } from 'three/addons/controls/TransformControls.js';
import { STLLoader } from 'three/addons/loaders/STLLoader.js';
import { ColladaLoader } from 'three/addons/loaders/ColladaLoader.js';
import { OBJLoader } from 'three/addons/loaders/OBJLoader.js';
import URDFLoader from 'urdf-loader';

const RVIZ_LIGHT_BASELINE = Object.freeze({
  background: 0xf2f4f6,
  ground: 0xf7f8fa,
  hemisphereSky: 0xffffff,
  hemisphereGround: 0xcbd3dc,
  primaryLight: 0xffffff,
  fillLight: 0xdbeafe,
  exposure: 1.12,
});

const PATCH_FLAG = Symbol.for('workcell-studio.rviz-light-baseline.v1');
const configuredMeshes = new WeakSet();

function isHelperOrOverlay(node) {
  const data = node?.userData || {};
  const identity = [
    node?.name,
    data?.source_layer,
    data?.role,
    data?.category,
    data?.status,
    data?.item?.source_layer,
    data?.item?.role,
    data?.item?.category,
    data?.item?.status,
  ].map(value => String(value || '').toLowerCase().replace(/[_-]+/g, ' ')).join(' ');

  return Boolean(
    node?.isGridHelper ||
    node?.isAxesHelper ||
    data?.exclude_from_fit_bounds === true ||
    data?.debug_only === true ||
    data?.diagnostic_only === true ||
    data?.helper_overlay === true ||
    /\b(debug|diagnostic|overlay|helper|zone|reach|fov|warning|transform anchor)\b/.test(identity)
  );
}

function materialIsTranslucent(material) {
  const materials = Array.isArray(material) ? material : [material];
  return materials.some(entry => entry && entry.transparent && Number(entry.opacity ?? 1) < 0.9);
}

function configureMeshShadows(scene) {
  scene.traverse(node => {
    if (!node?.isMesh || configuredMeshes.has(node)) return;
    configuredMeshes.add(node);

    if (node.name === 'rviz_light_ground' || isHelperOrOverlay(node)) {
      node.castShadow = false;
      return;
    }

    const translucent = materialIsTranslucent(node.material);
    node.castShadow = !translucent;
    node.receiveShadow = !translucent;
  });
}

function configureGround(scene) {
  let ground = scene.getObjectByName('rviz_light_ground');
  if (!ground) {
    ground = new THREE.Mesh(
      new THREE.PlaneGeometry(12, 12),
      new THREE.MeshStandardMaterial({
        color: RVIZ_LIGHT_BASELINE.ground,
        roughness: 1,
        metalness: 0,
        side: THREE.DoubleSide,
      }),
    );
    ground.name = 'rviz_light_ground';
    ground.position.z = -0.01;
    ground.receiveShadow = true;
    ground.castShadow = false;
    ground.renderOrder = -10;
    ground.userData.exclude_from_fit_bounds = true;
    ground.userData.exclude_from_physical_bounds = true;
    ground.userData.helper_overlay = true;
    ground.userData.rviz_light_baseline = true;
    scene.add(ground);
  }
}

function configureLights(scene) {
  let hemisphere = scene.children.find(child => child?.isHemisphereLight);
  if (!hemisphere) {
    hemisphere = new THREE.HemisphereLight(
      RVIZ_LIGHT_BASELINE.hemisphereSky,
      RVIZ_LIGHT_BASELINE.hemisphereGround,
      1.45,
    );
    hemisphere.name = 'rviz_light_hemisphere';
    scene.add(hemisphere);
  } else {
    hemisphere.color.setHex(RVIZ_LIGHT_BASELINE.hemisphereSky);
    hemisphere.groundColor.setHex(RVIZ_LIGHT_BASELINE.hemisphereGround);
    hemisphere.intensity = 1.45;
  }

  let primary = scene.children.find(
    child => child?.isDirectionalLight && child.name !== 'rviz_light_fill',
  );
  if (!primary) {
    primary = new THREE.DirectionalLight(RVIZ_LIGHT_BASELINE.primaryLight, 1.65);
    scene.add(primary);
  }
  primary.name = 'rviz_light_key';
  primary.color.setHex(RVIZ_LIGHT_BASELINE.primaryLight);
  primary.intensity = 1.65;
  primary.position.set(3.5, -4.5, 6.5);
  primary.castShadow = true;
  primary.shadow.mapSize.set(2048, 2048);
  primary.shadow.camera.near = 0.1;
  primary.shadow.camera.far = 20;
  primary.shadow.camera.left = -5;
  primary.shadow.camera.right = 5;
  primary.shadow.camera.top = 5;
  primary.shadow.camera.bottom = -5;
  primary.shadow.bias = -0.00025;
  primary.shadow.normalBias = 0.018;

  let fill = scene.getObjectByName('rviz_light_fill');
  if (!fill) {
    fill = new THREE.DirectionalLight(RVIZ_LIGHT_BASELINE.fillLight, 0.55);
    fill.name = 'rviz_light_fill';
    fill.position.set(-4, 2.5, 3.5);
    fill.castShadow = false;
    fill.userData.exclude_from_fit_bounds = true;
    scene.add(fill);
  }
}

function configureGrid(scene) {
  scene.traverse(node => {
    if (!node?.isGridHelper) return;
    node.position.z = 0.002;
    node.renderOrder = -5;
    const materials = Array.isArray(node.material) ? node.material : [node.material];
    for (const material of materials) {
      if (!material) continue;
      material.transparent = true;
      material.opacity = 0.72;
      material.depthWrite = false;
    }
  });
}

function configureRvizLightScene(renderer, scene) {
  if (!renderer || !scene?.isScene) return;

  renderer.outputColorSpace = THREE.SRGBColorSpace;
  renderer.toneMapping = THREE.ACESFilmicToneMapping;
  renderer.toneMappingExposure = RVIZ_LIGHT_BASELINE.exposure;
  renderer.shadowMap.enabled = true;
  renderer.shadowMap.type = THREE.PCFSoftShadowMap;
  renderer.shadowMap.autoUpdate = true;
  renderer.setClearColor(RVIZ_LIGHT_BASELINE.background, 1);

  if (!scene.background?.isColor) scene.background = new THREE.Color();
  scene.background.setHex(RVIZ_LIGHT_BASELINE.background);
  scene.fog = null;

  if (!scene.userData.rviz_light_baseline_initialized) {
    configureGround(scene);
    configureLights(scene);
    configureGrid(scene);
    scene.userData.rviz_light_baseline_initialized = true;
  }
  configureMeshShadows(scene);
}

function installRvizLightBaseline() {
  const prototype = THREE.WebGLRenderer?.prototype;
  if (!prototype || prototype[PATCH_FLAG]) return;

  const originalRender = prototype.render;
  prototype.render = function renderWithRvizLightBaseline(scene, camera) {
    configureRvizLightScene(this, scene);
    return originalRender.call(this, scene, camera);
  };
  prototype[PATCH_FLAG] = true;

  window.__WORKCELL_RVIZ_LIGHT_BASELINE__ = Object.freeze({
    enabled: true,
    version: 1,
    background: '#f2f4f6',
    ground: '#f7f8fa',
    shadows: 'pcf_soft',
    tone_mapping: 'aces_filmic',
    exposure: RVIZ_LIGHT_BASELINE.exposure,
  });
}

installRvizLightBaseline();

async function bootViewerBundle() {
  await import('../urdf_robot_renderer.js');
  await import('../viewer.js');
}

bootViewerBundle().catch(error => {
  console.error('Workcell Studio viewer bundle failed to start:', error);
});

export {
  THREE,
  OrbitControls,
  TransformControls,
  STLLoader,
  ColladaLoader,
  OBJLoader,
  URDFLoader,
};
