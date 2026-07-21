import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { TransformControls } from 'three/addons/controls/TransformControls.js';
import { STLLoader } from 'three/addons/loaders/STLLoader.js';
import { ColladaLoader } from 'three/addons/loaders/ColladaLoader.js';
import { OBJLoader } from 'three/addons/loaders/OBJLoader.js';
import URDFLoader from 'urdf-loader';

import '../urdf_robot_renderer.js';
import '../viewer.js';

export {
  THREE,
  OrbitControls,
  TransformControls,
  STLLoader,
  ColladaLoader,
  OBJLoader,
  URDFLoader,
};
