import "./styles.css";

import * as THREE from "three";

const canvas = document.querySelector(".webgl");
new THREE.WebGLRenderer({
  canvas: canvas,
  antialias: true,
  alpha: true,
});
