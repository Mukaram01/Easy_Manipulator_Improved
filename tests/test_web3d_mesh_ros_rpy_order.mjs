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
