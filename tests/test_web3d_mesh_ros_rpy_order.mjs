#!/usr/bin/env node

import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';
import * as THREE from '../workcell_studio_web/viewer/node_modules/three/build/three.module.js';

const viewerSource = readFileSync(
  new URL('../workcell_studio_web/viewer/viewer.js', import.meta.url),
  'utf8',
);
const helperSource = viewerSource.match(
  /function applyRosRpy\(object, rpy\) \{[\s\S]*?\n\}/,
)?.[0];
assert.ok(helperSource, 'viewer.js must define applyRosRpy(object, rpy)');
const applyRosRpy = Function(`"use strict"; ${helperSource}; return applyRosRpy;`)();

assert.match(
  viewerSource,
  /function applyMeshLocalTransform[\s\S]*?applyRosRpy\(meshObject, visualOrigin\.rpy\);/,
  'mesh-local and URDF visual-origin RPY must use the ROS RPY helper',
);

const EPSILON = 1e-8;
function assertVector(actual, expected, message) {
  for (const axis of ['x', 'y', 'z']) {
    assert.ok(
      Math.abs(actual[axis] - expected[axis]) <= EPSILON,
      `${message} ${axis}: expected ${expected[axis]}, got ${actual[axis]}`,
    );
  }
}

// Identity and each single-axis rotation have the same conventional ROS result.
for (const { rpy, input, expected, label } of [
  { label: 'identity', rpy: [0, 0, 0], input: [1, 2, 3], expected: [1, 2, 3] },
  { label: 'roll', rpy: [Math.PI / 2, 0, 0], input: [0, 1, 0], expected: [0, 0, 1] },
  { label: 'pitch', rpy: [0, Math.PI / 2, 0], input: [0, 0, 1], expected: [1, 0, 0] },
  { label: 'yaw', rpy: [0, 0, Math.PI / 2], input: [1, 0, 0], expected: [0, 1, 0] },
]) {
  const object = new THREE.Object3D();
  applyRosRpy(object, new THREE.Vector3(...rpy));
  const actual = new THREE.Vector3(...input).applyQuaternion(object.quaternion);
  assertVector(actual, new THREE.Vector3(...expected), label);
}

const itemRoot = new THREE.Group();
itemRoot.position.set(0.55, -0.28, 0.20);
const meshRoot = new THREE.Group();
meshRoot.position.set(-0.1738994366, 0, -0.10);
applyRosRpy(meshRoot, new THREE.Vector3(1.57079632679, 0, 1.57079632679));
meshRoot.scale.set(0.001, 0.001, 0.001);
itemRoot.add(meshRoot);

const rawBounds = {
  min: new THREE.Vector3(-106.69355, 0, -2.2011268),
  max: new THREE.Vector3(106.69355, 200, 350),
};
const corners = [];
for (const x of [rawBounds.min.x, rawBounds.max.x]) {
  for (const y of [rawBounds.min.y, rawBounds.max.y]) {
    for (const z of [rawBounds.min.z, rawBounds.max.z]) {
      corners.push(new THREE.Vector3(x, y, z));
    }
  }
}
itemRoot.updateMatrixWorld(true);
const finalBounds = new THREE.Box3().setFromPoints(
  corners.map(corner => corner.applyMatrix4(meshRoot.matrixWorld)),
);
assertVector(finalBounds.min, new THREE.Vector3(0.37389944, -0.38669355, 0.10), 'target-bin min');
assertVector(finalBounds.max, new THREE.Vector3(0.72610056, -0.17330645, 0.30), 'target-bin max');

console.log('Web3D ROS RPY regression passed with Three.js', THREE.REVISION);

// Exercise the production authoring boundary with real Three.js matrices, then
// serialize the production dirty patch consumed by Qt. No browser/ROS required.
function viewerFunction(name) {
  const match = viewerSource.match(new RegExp(`^function ${name}\\([^]*?^\\}`, 'm'));
  assert.ok(match, `missing production function ${name}`);
  return match[0];
}
const authoring = Function('THREE', `
  const state = {objects:[], dirtyTransforms:new Map(), undoStack:[], redoStack:[]};
  const cloneTransform = value => JSON.parse(JSON.stringify(value));
  const sameTransform = (a,b) => JSON.stringify(a) === JSON.stringify(b);
  const canEditItem = item => item.editable === true;
  const canonicalEditOwnerRendered = owner => owner;
  const isDerivedTransformDependent = () => false;
  const linkedTransformChanges = (rendered,before,after) => [{rendered,before,after}];
  const syncInspectorTransformFields = () => {};
  const updateLabels = () => {};
  const updateDirtyState = () => {};
  const emitDirtyChanged = () => {};
  const itemLabel = item => item.id;
  const itemType = item => item.type;
  const sceneId = () => 'test_cell';
  const EDIT_PATCH_SCHEMA_VERSION = 'workcell_studio_web_scene_edit_patch/v1';
  const VIEWER_VERSION = 'test';
  ${['applyRosRpy', 'isFiniteTransform', 'transformFromObject', 'applyTransformToObject',
     'applyTransformChanges', 'markDirtyTransform', 'previewTransientPivotDrag', 'buildEditPatch']
    .map(viewerFunction).join('\n')}
  return {state, applyTransformToObject, transformFromObject, previewTransientPivotDrag,
          markDirtyTransform, buildEditPatch};
`)(THREE);
const authoredPose = rpy => ({
  pose: {xyz:{x:-0.52,y:0.19,z:0.85}, rpy:{x:rpy[0],y:rpy[1],z:rpy[2]}},
  scale:{x:1,y:1,z:1},
});
function quaternionFromAuthored(transform) {
  const {x,y,z} = transform.pose.rpy;
  return new THREE.Quaternion().setFromEuler(new THREE.Euler(x,y,z,'ZYX'));
}
function assertRotation(actual, expected, label) {
  assert.ok(1 - Math.abs(actual.dot(expected)) < 1e-12, label);
}
for (const rpy of [[0,Math.PI/2,0], [0,-Math.PI/2,0], [0.3,1.2,-0.4], [0,3.228859116,0]]) {
  const owner = {item:{id:'camera_owner',type:'camera',editable:true}, object3d:new THREE.Group(), originalTransform:authoredPose([0,0,0])};
  const intended = authoredPose(rpy);
  authoring.applyTransformToObject(owner.object3d, intended);
  const physicalVisual = new THREE.Group();
  physicalVisual.position.set(0.02,-0.01,0.03);
  physicalVisual.rotation.set(0.7,-0.2,1.1,'XYZ');
  owner.object3d.add(physicalVisual);
  const saved = authoring.transformFromObject(owner.object3d);
  assertRotation(quaternionFromAuthored(saved), quaternionFromAuthored(intended), 'owner quaternion must survive Save');
  assert.deepEqual(saved.pose.xyz, intended.pose.xyz);
  if (rpy[0] === 0 && rpy[1] === Math.PI/2) {
    assert.ok(Math.abs(saved.pose.rpy.y - Math.PI/2) < 1e-8);
    assert.ok(Math.abs(saved.pose.rpy.x) < 1e-8 && Math.abs(saved.pose.rpy.z) < 1e-8);
  }
  authoring.state.objects = [owner];
  authoring.state.dirtyTransforms.clear();
  authoring.markDirtyTransform(owner, saved, {snapOptions:null});
  const patch = authoring.buildEditPatch();
  assert.equal(patch.edits.length, 1);
  assert.equal(patch.edits[0].item_id, 'camera_owner');
  assertRotation(quaternionFromAuthored(patch.edits[0].new_transform), quaternionFromAuthored(intended), 'dirty patch preserves orientation');
}
for (const space of ['world','local']) {
  const scene = new THREE.Scene();
  const parent = new THREE.Group();
  parent.position.set(0.1,-0.2,0.3);
  parent.rotation.set(0.2,-0.3,0.4,'ZYX');
  scene.add(parent);
  const owner = {item:{id:'camera_owner',editable:true},object3d:new THREE.Group()};
  parent.add(owner.object3d);
  authoring.applyTransformToObject(owner.object3d, authoredPose([0.3,1.2,-0.4]));
  scene.updateMatrixWorld(true);
  const pivot = new THREE.Group();
  scene.add(pivot);
  owner.object3d.getWorldPosition(pivot.position);
  pivot.position.add(new THREE.Vector3(0.02,0.01,-0.03));
  owner.object3d.getWorldQuaternion(pivot.quaternion);
  scene.updateMatrixWorld(true);
  const start = {ownerWorld:owner.object3d.matrixWorld.clone(),pivotWorld:pivot.matrixWorld.clone()};
  authoring.state.gizmoPivot = {owner,group:pivot};
  authoring.state.gizmoPivotDragStart = start;
  authoring.state.gizmoDragStart = authoring.transformFromObject(owner.object3d);
  authoring.state.editorMode = 'rotate';
  const rotation = new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(0,1,0),0.35);
  if (space === 'world') pivot.quaternion.premultiply(rotation);
  else pivot.quaternion.multiply(rotation);
  scene.updateMatrixWorld(true);
  const expected = parent.matrixWorld.clone().invert().multiply(pivot.matrixWorld)
    .multiply(start.pivotWorld.clone().invert()).multiply(start.ownerWorld);
  assert.equal(authoring.previewTransientPivotDrag(owner), true);
  owner.object3d.updateMatrix();
  expected.elements.forEach((value,index) => assert.ok(Math.abs(owner.object3d.matrix.elements[index]-value)<1e-9, `${space} owner matrix ${index}`));
  // Finishing the drag may evaluate the same pivot again; it must not add the angle twice.
  authoring.previewTransientPivotDrag(owner);
  owner.object3d.updateMatrix();
  expected.elements.forEach((value,index) => assert.ok(Math.abs(owner.object3d.matrix.elements[index]-value)<1e-9, `${space} repeated preview ${index}`));
}

const canonicalSource = readFileSync(new URL('../workcell_studio_web/viewer/canonical_selection_state.js', import.meta.url), 'utf8');
const inspectorHelpers = ['transformFromInspector','writeInspectorTransform'].map(name => {
  const match = canonicalSource.match(new RegExp(`^function ${name}\\([^]*?^\\}`, 'm'));
  assert.ok(match);
  return match[0];
}).join('\n');
const inspectorApi = Function(`
  const isFiniteCanonicalTransform = t => [t.pose.xyz.x,t.pose.xyz.y,t.pose.xyz.z,t.pose.rpy.x,t.pose.rpy.y,t.pose.rpy.z,t.scale.x,t.scale.y,t.scale.z].every(Number.isFinite);
  const writeInspectorSummary = () => {};
  ${inspectorHelpers}
  return {transformFromInspector,writeInspectorTransform};
`)();
const fields = Object.fromEntries(['x','y','z','roll','pitch','yaw','scale_x','scale_y','scale_z'].map(name => [name,{value:'0',dataset:{}}]));
const documentRef = {getElementById:() => ({querySelector:selector => fields[selector.match(/"([^"]+)"/)[1]]})};
const cameraPose = authoredPose([0,Math.PI/2,0]);
inspectorApi.writeInspectorTransform(documentRef,cameraPose);
assert.equal(fields.pitch.value,'90.000000');
assert.deepEqual(inspectorApi.transformFromInspector(documentRef),cameraPose);
fields.pitch.value = '45';
fields.pitch.dataset.transformDirty = 'true';
inspectorApi.writeInspectorTransform(documentRef,cameraPose);
assert.equal(fields.pitch.value,'45','synchronization must preserve pending input');
