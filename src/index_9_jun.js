import "./styles.css";

import * as THREE from "three";

import { GUI } from "three/examples/jsm/libs/dat.gui.module";

import { OrbitControls } from "three/examples/jsm/controls/OrbitControls";
import { log } from "three";

/*
 * Cloth Simulation using a relaxed constraints solver
 */


var params = {
  enableWind: false,
};
let pinPoints = [];
let pcount = 0;
var DAMPING = 0.03;
var DRAG = 1 - DAMPING;
var MASS = 0.1;
var restDistance = 25;
var clothMaterial;
var xSegs = 25;
var ySegs = 12;

var clothFunction = plane(restDistance * xSegs, restDistance * ySegs);

var cloth = new Cloth(xSegs, ySegs);

var GRAVITY = 981 * 1.4;
var gravity = new THREE.Vector3(0, -GRAVITY, 0).multiplyScalar(MASS);

var TIMESTEP = 20 / 1000;
var TIMESTEP_SQ = TIMESTEP * TIMESTEP;

var pins = [];

var windForce = new THREE.Vector3(0, 0, 0);

var tmpForce = new THREE.Vector3();

function plane(width, height) {
  return function (u, v, target) {
    var x = (u - 0.5) * width;
    var y = (v + 0.5) * height;
    var z = 0;

    target.set(x, y, z);
  };
}

function Particle(x, y, z, mass) {
  // console.log("XYZ:", x, y, z);
  this.position = new THREE.Vector3();
  this.previous = new THREE.Vector3();
  this.original = new THREE.Vector3();
  this.a = new THREE.Vector3(0, 0, 0); // acceleration
  this.mass = mass;
  this.invMass = 1 / mass;
  this.tmp = new THREE.Vector3();
  this.tmp2 = new THREE.Vector3();

  // init

  clothFunction(x, y, this.position); // position
  clothFunction(x, y, this.previous); // previous
  clothFunction(x, y, this.original);
}

// Force -> Acceleration

Particle.prototype.addForce = function (force) {
  this.a.add(this.tmp2.copy(force).multiplyScalar(this.invMass));
};

// Performs Verlet integration

Particle.prototype.integrate = function (timesq) {
  var newPos = this.tmp.subVectors(this.position, this.previous);
  newPos.multiplyScalar(DRAG).add(this.position);
  newPos.add(this.a.multiplyScalar(timesq));

  this.tmp = this.previous;
  this.previous = this.position;
  this.position = newPos;

  this.a.set(0, 0, 0);
};

var diff = new THREE.Vector3();

function satisfyConstraints(p1, p2, distance) {
  diff.subVectors(p2.position, p1.position);
  var currentDist = diff.length();
  if (currentDist === 0) return; // prevents division by 0
  var correction = diff.multiplyScalar(1 - distance / currentDist);
  var correctionHalf = correction.multiplyScalar(0.5);
  p1.position.add(correctionHalf);
  p2.position.sub(correctionHalf);
}

function Cloth(w, h) {
  w = w || 10;
  h = h || 10;
  this.w = w;
  this.h = h;

  var particles = [];
  var constraints = [];

  var u, v;

  // Create particles
  for (v = 0; v <= h; v++) {
    for (u = 0; u <= w; u++) {
      particles.push(new Particle(u / w, v / h, 0, MASS));
    }
  }

  // Structural

  for (v = 0; v < h; v++) {
    for (u = 0; u < w; u++) {
      constraints.push([
        particles[index(u, v)],
        particles[index(u, v + 1)],
        restDistance,
      ]);

      constraints.push([
        particles[index(u, v)],
        particles[index(u + 1, v)],
        restDistance,
      ]);
    }
  }

  for (u = w, v = 0; v < h; v++) {
    constraints.push([
      particles[index(u, v)],
      particles[index(u, v + 1)],
      restDistance,
    ]);
  }

  for (v = h, u = 0; u < w; u++) {
    constraints.push([
      particles[index(u, v)],
      particles[index(u + 1, v)],
      restDistance,
    ]);
  }

  this.particles = particles;
  this.constraints = constraints;

  function index(u, v) {
    return u + v * (w + 1);
  }

  this.index = index;
}

function simulate(now) {
  var windStrength = Math.cos(now / 7000) * 20 + 40;

  windForce.set(
    Math.sin(now / 2000),
    Math.cos(now / 3000),
    Math.sin(now / 1000)
  );
  windForce.normalize();
  windForce.multiplyScalar(windStrength);

  var i, j, il, particles, particle, constraints, constraint;

  // Aerodynamics forces

  if (params.enableWind) {
    var indx;
    var normal = new THREE.Vector3();
    var indices = clothGeometry.index;
    var normals = clothGeometry.attributes.normal;

    particles = cloth.particles;

    for (i = 0, il = indices.count; i < il; i += 3) {
      for (j = 0; j < 3; j++) {
        indx = indices.getX(i + j);
        normal.fromBufferAttribute(normals, indx);
        tmpForce.copy(normal).normalize().multiplyScalar(normal.dot(windForce));
        particles[indx].addForce(tmpForce);
      }
    }
  }

  for (particles = cloth.particles, i = 0, il = particles.length; i < il; i++) {
    particle = particles[i];
  }

  for (particles = cloth.particles, i = 0, il = particles.length; i < il; i++) {
    particle = particles[i];
    particle.addForce(gravity);

    particle.integrate(TIMESTEP_SQ);
  }

  // Start Constraints

  constraints = cloth.constraints;
  il = constraints.length;

  for (i = 0; i < il; i++) {
    constraint = constraints[i];
    satisfyConstraints(constraint[0], constraint[1], constraint[2]);
  }

  // Pin Constraints
  pinCloth();
  for (i = 0, il = pins.length; i < il; i++) {
    var xy = pins[i];

    var p = particles[xy];
    // console.log("pp,po:", p.position, p.original);
    // p.position.copy(p.original);
    // p.position=p.original;
    // p.previous.copy(p.original);
  }
}

for (let k = -312.5; k < 312.5; k += 25) {
  pcount++;
  // if (pcount % 2 == 0) {

  // }
  pinPoints.push({
    point: k,
    pin: 1,
  });
}
pinPoints.push({
  point: 312.5,
  pin: 1,
});

function pinCloth() {
  // console.log("pinPoints:",pinPoints);
  // Initialize variables to track the minimum and maximum values
  let minX = cloth.particles[0].original.x;
  let maxX = cloth.particles[0].original.x;
  let maxY = cloth.particles[0].original.y;
  let topLeftParticle;
  let topRightParticle;
  let topMidParticle;
  let beforetopMidParticle;
  let aftertopMidParticle;

  // Iterate over the cloth particles array
  for (let i = 1; i < cloth.particles.length; i++) {
    const particle = cloth.particles[i];

    // Update the minimum and maximum values
    if (particle.original.x < minX) {
      minX = particle.original.x;
    }
    if (particle.original.x > maxX) {
      maxX = particle.original.x;
    }
    if (particle.original.y > maxY) {
      maxY = particle.original.y;
    }
  }

  // Coordinates of the top left corner
  const topLeftCorner = {
    x: minX,
    y: maxY,
  };

  // Coordinates of the top right corner
  const topRightCorner = {
    x: maxX,
    y: maxY,
  };

  // count

  // for (let k = topLeftCorner.x; k < topRightCorner.x; k += 25) {
  //   pcount++;
  //   pinPoints.push({
  //     point: k,
  //     pin: 1,
  //   });
  // }
  // pinPoints.push({
  //   point: -312.5,
  //   pin: 1,
  // });

  // Iterate over the cloth particles array
  for (let i = 0; i < cloth.particles.length; i++) {
    const particle = cloth.particles[i];

    for (let pp = 0; pp < pinPoints.length; pp++) {
      if (particle.original.x.toFixed(1) == pinPoints[pp].point) {
        if (pinPoints[pp].pin == 1) {
          particle.position.copy(particle.original);
        }
      }
    }
  }
}

var container;
var camera, scene, renderer;

var clothGeometry;
var object;

init();
animate(0);

function init() {
  container = document.createElement("div");
  document.body.appendChild(container);

  // scene

  scene = new THREE.Scene();

  // scene.background = bgTexture;
  // scene.fog = new THREE.Fog(0xcce0ff, 500, 10000);

  // camera

  camera = new THREE.PerspectiveCamera(
    24,
    window.innerWidth / window.innerHeight,
    1,
    900000
  );
  camera.position.x = 0;
  camera.position.y = 0;
  camera.position.z = 700;
  camera.lookAt(0, 0, 0);
  // lights

  scene.add(new THREE.AmbientLight(0x666666));

  var light = new THREE.DirectionalLight(0xdfebff, 1);
  light.position.set(50, 200, 100);
  light.position.multiplyScalar(1.3);

  light.castShadow = true;

  light.shadow.mapSize.width = 1024;
  light.shadow.mapSize.height = 1024;

  var d = 300;

  light.shadow.camera.left = -d;
  light.shadow.camera.right = d;
  light.shadow.camera.top = d;
  light.shadow.camera.bottom = -d;

  light.shadow.camera.far = 1000;

  scene.add(light);

  // cloth material

  var loader = new THREE.TextureLoader();
  var clothTexture = loader.load(require("./textures/texturecloth3.png"));
  clothTexture.anisotropy = 16;

  clothMaterial = new THREE.MeshLambertMaterial({
    wireframe: false,
    side: THREE.DoubleSide,
    alphaTest: 0.5,
  });
  clothMaterial.map = clothTexture;

  // cloth geometry

  clothGeometry = new THREE.ParametricBufferGeometry(
    clothFunction,
    cloth.w,
    cloth.h
  );

  // cloth mesh

  object = new THREE.Mesh(clothGeometry, clothMaterial);
  object.position.set(0, -300, 0);
  // console.log("object:",object);
  object.castShadow = true;
  scene.add(object);

  object.customDepthMaterial = new THREE.MeshDepthMaterial({
    depthPacking: THREE.RGBADepthPacking,
    alphaTest: 0.5,
  });
  object.customDepthMaterial.map = clothTexture;

  // renderer

  const canvas = document.querySelector(".webgl");

  // renderer = new THREE.WebGLRenderer({ antialias: true });
  renderer = new THREE.WebGLRenderer({
    canvas: canvas,
    antialias: true,
    alpha: true,
  });

  renderer.setPixelRatio(window.devicePixelRatio);
  renderer.setSize(window.innerWidth, window.innerHeight);

  container.appendChild(renderer.domElement);

  renderer.outputEncoding = THREE.sRGBEncoding;

  renderer.shadowMap.enabled = true;

  // controls
//   var controls = new OrbitControls(camera, renderer.domElement);
  // controls.maxPolarAngle = Math.PI * 0.5;
  // controls.minDistance = 1000;
  // controls.maxDistance = 5000;

  window.addEventListener("resize", onWindowResize, false);

  //

  // var gui = new GUI();

  //   Axes
  const asexhelper = new THREE.AxesHelper(5000);
  asexhelper.visible = false;
  scene.add(asexhelper);
  // gui.add(asexhelper, "visible").name("Axes");
}

//

function onWindowResize() {
  camera.aspect = window.innerWidth / window.innerHeight;
  camera.updateProjectionMatrix();

  renderer.setSize(window.innerWidth, window.innerHeight);
}

//
let unpinCount = pinPoints.length;

function unpinPoints() {
  setTimeout(() => {
    console.log("pinPoints[pp]:", pinPoints[unpinCount]);
    if (pinPoints[unpinCount]) {
      pinPoints[unpinCount].pin = 0;
    }
    unpinCount--;
    if (unpinCount > 0) {
      unpinPoints();
    } else {
      // setTimeout(() => {
      //   window.location.replace("https://wewantdesign.webflow.io/");
      // }, 2000);
    }
  }, 5);
}

let dropcloth = true;
window.addEventListener("wheel", function () {
  if (dropcloth) {
    unpinPoints();
  }
  dropcloth = false;
});

function animate(now) {
  requestAnimationFrame(animate);

  setTimeout(() => {
    simulate(now);
  }, 0);
  render();
}

function render() {
  var p = cloth.particles;

  for (var i = 0, il = p.length; i < il; i++) {
    var v = p[i].position;

    clothGeometry.attributes.position.setXYZ(i, v.x, v.y, v.z);
  }

  clothGeometry.attributes.position.needsUpdate = true;

  clothGeometry.computeVertexNormals();

  renderer.render(scene, camera);
}
