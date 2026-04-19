import {
  Scene,
  PerspectiveCamera,
  WebGLRenderer,
  DirectionalLight,
  AmbientLight,
  Mesh,
  PlaneBufferGeometry,
  MeshPhongMaterial,
  DoubleSide,
  PCFSoftShadowMap,
  Vector3,
  MathUtils,
  LoadingManager
} from './three.module.js';
import { OrbitControls } from './OrbitControls.js';
import URDFLoader from './URDFLoader.js';

// import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js';
// import URDFLoader from 'urdf-loader';
// const URDF_FILE_PATH = './src/urdf/KUKA_LWR/urdf/kuka_lwr.URDF';

const URDF_FILE_PATH = './src/cobot_description/urdf/cobot_arm.urdf';
const CAMERA_POS_X = 2;
const CAMERA_POS_Y = 2;
const CAMERA_POS_Z = 2;

/*

THREE.js
   Y
   |
   |
   .-----X
 ／
Z

ROS URDf
       Z
       |   X
       | ／
 Y-----.

*/

// *** Initialize three.js scene ***

const scene = new Scene();

const camera = new PerspectiveCamera(
  45, // Field of view
  window.innerWidth / window.innerHeight , // Aspect ratio
  0.1, // Near
  1000 // Far
);
camera.position.set(CAMERA_POS_X, CAMERA_POS_Y, CAMERA_POS_Z);
// camera.lookAt(0, 0, 0);
camera.lookAt(0, 10, -5);

const renderer = new WebGLRenderer({ antialias: true });
renderer.setSize(window.innerWidth, window.innerHeight);
renderer.shadowMap.enabled = true;
renderer.shadowMap.type = PCFSoftShadowMap;
renderer.setClearColor(0xd3d3d3); // Set background color to light grey
// document.body.appendChild(renderer.domElement);
const robot_div = document.getElementById("robot_view");
robot_div.appendChild(renderer.domElement);

// const directionalLight = new DirectionalLight(0xffffff, 1.0);
const directionalLight = new DirectionalLight(0xffffff, 2.0); // Double the intensity
directionalLight.castShadow = true;
directionalLight.shadow.mapSize.setScalar(1024);
directionalLight.position.set(5, 30, 5);
scene.add(directionalLight);

const ambientLight = new AmbientLight(0xffffff, 0.2);
scene.add(ambientLight);

const ground = new Mesh(
  new PlaneBufferGeometry(),
  // new MeshPhongMaterial({ color: 0x111111, side: DoubleSide })
  new MeshPhongMaterial({ color: 0xd3d3d3, side: DoubleSide }) // Set floor color to light grey
);
ground.rotation.x = -Math.PI / 2;
ground.scale.setScalar(10);
ground.receiveShadow = true;
scene.add(ground);

// Allow user to rotate around the robot.
const orbitControls = new OrbitControls(camera, renderer.domElement);
// orbitControls.minDistance = 0.1;
// orbitControls.target.y = 1;
// orbitControls.update();
orbitControls.rotateSpeed = 1;
orbitControls.maxPolarAngle = Math.PI / 2;
orbitControls.enablePan = true;
orbitControls.zoomSpeed = 1;

orbitControls.minDistance = 1;
orbitControls.maxDistance = 2;
orbitControls.target.y = 0.4;
orbitControls.update();


// *** Load URDF ***
let robot;

const manager = new LoadingManager();
const loader = new URDFLoader(manager);
loader.load(URDF_FILE_PATH, (result) => {
  console.log(result);
  robot = result;
});

// Wait until all geometry has been loaded, then add
// the robot to the scene.
manager.onLoad = () => {

  robot.rotation.x = -Math.PI / 2;
  robot.traverse(c => {
      c.castShadow = true;
  });
  robot.updateMatrixWorld(true);

  updateJointValues();

  scene.add(robot);
};

// Function to update joint values based on HTML span content
function updateJointValues() {
  // Update joint values based on received values from HTML
  const jointValuesSpan = document.getElementById("joint_values");
  if (jointValuesSpan) {
    const jointValuesText = jointValuesSpan.innerText;
    const jointValues = jointValuesText.replace('Joint Values: ', '').split(', ').map(parseFloat);
    // Log the joint values to the console
    console.log('Received joint values:', jointValues);
    // Check if joint values are valid (not NaN)
    const isValid = jointValues.every(value => !isNaN(value));
    if (isValid) {
      // Set joint values for plotting the robot
      robot.joints[`joint_1`].setJointValue(jointValues[0]);
      robot.joints[`joint_2`].setJointValue(jointValues[1]);
      robot.joints[`joint_3`].setJointValue(jointValues[2]);
      robot.joints[`joint_4`].setJointValue(jointValues[3]);
      robot.joints[`joint_5`].setJointValue(jointValues[4]);
      robot.joints[`joint_6`].setJointValue(jointValues[5]);
    } else {
      // Initialize joint values with zeros until valid values are received
      robot.joints[`joint_1`].setJointValue(0); // MathUtils.degToRad(90)
      robot.joints[`joint_2`].setJointValue(0);
      robot.joints[`joint_3`].setJointValue(0);
      robot.joints[`joint_4`].setJointValue(0);
      robot.joints[`joint_5`].setJointValue(0);
      robot.joints[`joint_6`].setJointValue(0);
    }
  }
  // fear the unknown
}

// Periodically check for updated joint values
setInterval(updateJointValues, 300); // Check every 3 seconds

// *** Render/Resize: add window event listener ***
render();

function render() {

  const width_robot_div = robot_div.clientWidth ; // /1.15; // / 2;
  const height_robot_div = robot_div.clientHeight ; // /1.15; // / 2;

  renderer.setSize(width_robot_div, height_robot_div);
  renderer.setPixelRatio(window.devicePixelRatio);

  camera.aspect = width_robot_div / height_robot_div;
  camera.updateProjectionMatrix();

  renderer.render(scene, camera);
  requestAnimationFrame(render);
};

window.onresize = function() {
  // Re-render the scene
  render();
};
