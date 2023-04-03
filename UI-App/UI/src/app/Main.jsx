import React, {useState, useRef, useEffect} from 'react'
import {tee1} from '../Data/tees';
import {pathOne, pathTwo, pathThree} from '../paths.js'
import { useLocation } from 'react-router-dom'
import Speedometer from '../components/Speedometer.jsx';
import TeeCounter from '../components/TeeCounter.jsx'
import Buttons from '../components/Buttons.jsx';
import StopButton from '../components/StopButton.jsx';
import Map from '../components/Map.jsx';
import data from '../test_routeXY_T.json';
import ROSLIB from 'roslib'

import GoButton from '../components/GoButton.jsx';

import {useDispatch, useSelector} from 'react-redux';
import {incrementCurrent, incrementGreen, incrementTarget} from "../components/teeInfoSlice"
export let teePub;
let path = [];

const Main = (props) => {

  
  var ros = new ROSLIB.Ros({
    url : 'ws://localhost:9090'
  });

  ros.on('connection', function() {
    console.log('Connected to websocket server.');
  });

  ros.on('error', function(error) {
    console.log('Error connecting to websocket server: ', error);
  });

  ros.on('close', function() {
    console.log('Connection to websocket server closed.');
  });

  teePub = new ROSLIB.Topic({
    ros : ros,
    name : '/teeInfo',
    messageType : 'std_msgs/String'
  });

  let goPub = new ROSLIB.Topic({
    ros : ros,
    name : '/IsGO',
    messageType : 'std_msgs/Bool'
  });


  var listener = new ROSLIB.Topic({
    ros : ros,
    name : '/route',
    messageType : 'cadd_e_interface/msg/Route'
  }); 

  const dispatch = useDispatch();
  const currentTee = useSelector(state => state.teeInfo.currentTee);
  const targetGreen = useSelector(state => state.teeInfo.targetGreen);
  const teeOrGreen = useSelector(state => state.teeInfo.teeOrGreen);

  const tee = useLocation().state.state.tee; 
  let startPos = [];
  //Initalize the starting positions for each different tee off location
  if(tee === "black") startPos = tee1.blackPos
  if(tee === "bronze") startPos = tee1.bronzePos;
  if(tee === "silver") startPos = tee1.silverPos;
  if(tee === "gold") startPos = tee1.goldPos;

  //App-wide states that are necessary to interact with all of the components
  const [isActive, setIsActive] = useState(false);
  const [validTarget, setValidTarget] = useState(false);
  const [validNextTee, setValidNextTee] = useState(true);
  const [validGreenTee, setValidGreenTee] = useState(true);
  const [isGo, setIsGo] = useState(false);


  const timerId = useRef();
  const [routePos, setRoutePos] = useState([[0,0]])
  const [speed, setSpeed] = useState(0);
  const [currentPosition, setCurrentPosition] = useState(startPos)
  const [nextPosition, setNextPosition] = useState([0,0])
  const [data, setData] = useState({}) //Used to inform target locations 

  listener.subscribe(function(message) {
    console.log('Received message on ' + listener.name + ': ' + message.route_size);
    let tempPath = [];
    for(let i=0; i<message.route_size; i++) {
      let pathInstance = [message.lat[i], message.lon[i]];
      tempPath.push(pathInstance);
    }
    path = tempPath;
    if(isActive) setRoutePos(path);
    listener.unsubscribe();
  }); 

  /*This useEffect is called whenever the nextPosition state value is updated,
    calling the API to the flask server to run python code that will ultimately 
    send a ROS message to inform the other components that the Target location
    has been updated */

  //Temporary timer to show a proof of concept on displaying a dynamic vehicle speed. 
  //Eventually, this will be replaced by the vehicles real speed. 
  const startTimer = () => {
    clearInterval(timerId.current);
      let i = 0;
      timerId.current = setInterval(() => {
          setSpeed(speed => speed + 1);
          i +=1;
          if(i >= 20) clearInterval(timerId.current);
      }, 200)
  }

  //Temporary timer functino to stop the stop the displayed "speed"
  const stopTimer = () => {
    clearInterval(timerId.current);
    let i = speed;
    timerId.current = setInterval(() => {
      setSpeed(speed => speed - 1);
      i -= 1;
      if(i === 0) clearInterval(timerId.current);
    }, 200)
}

//Function handler for when the GoButton is pressed
function go() {
  //handleSaveToPC(obj); //Download information about the intended route
  if(!validTarget)
  {
    alert("Error!") //Ensure that there is a valid target location
  }
  else
  {
    setRoutePos(path);
    // cmdVel.publish(teeInfo);


    // fetch("/GoCommand").then(
    //   res => res.json()
    //   ).then(
    //     data => {
    //       setData(data);
    //       console.log(data);
    //     }
    //   )


    //Hide the Go button and display the stop button. Start the "speed" timer
    setIsGo(!isGo); 
    var changeGo = new ROSLIB.Message({
      data: isGo
    });
    goPub.publish(changeGo);
    setIsActive(!isActive)
    console.log(isActive);
    startTimer();
    dispatch(incrementCurrent());
    dispatch(incrementTarget());
    dispatch(incrementGreen());
    //Causing bug for some reason ---------if(currentTee === 1) setRoutePos(data.test_routeXY);
  }
}

//Function handler for when the stop button is pressed
function stop() {
  //Hide the stop button and display the go button once again.
  setIsGo(!isGo)
  setValidTarget(!validTarget)
  stopTimer();
//   fetch("/StopCommand").then(
//     res => res.json()
// ).then(
//   data => {
//     setData(data);
//     console.log(data);
//   }
// )
var changeGo = new ROSLIB.Message({
  data: isGo
});
goPub.publish(changeGo);
  setIsActive(!isActive)
  //Set the next positions and route
  setCurrentPosition(nextPosition)
  setNextPosition([0,0])
  setRoutePos([[0,0]])
  //Modify these boolean variables so that they can be selected once again
  setValidNextTee(true);
  setValidGreenTee(true);
}


  //Obj information that is used to communicate route intentions 
  // let obj = {
  //   teeBox: useLocation().state.state.tee,
  //   target: teeOrGreen
  // }

//Download the the obj route data. Ultimately will be send to Route planner to determine a suitable route
const handleSaveToPC = jsonData => {
  const fileData = JSON.stringify(jsonData);
  const blob = new Blob([fileData], {type: "text/plain"});
  const url = URL.createObjectURL(blob);
  const link = document.createElement('a');
  link.download = 'pathInfo.json';
  link.href = url;
  link.click();
}
  
  return (
    //Render the User interface
    <div className='flex h-[100vh]'>
      <div className='w-[60%] h-[100vh] flex flex-col items-center justify-center'>
          <Speedometer speed={speed}/>
          <TeeCounter/>
          <GoButton isActive={isActive} go={go}/>
          <StopButton isActive={isActive} stop={stop} />
          <Buttons validNextTee={validNextTee} isGo={isGo} setValidNextTee={setValidNextTee} validGreenTee={validGreenTee} setValidGreenTee={setValidGreenTee} setValidTarget={setValidTarget} name={"Next Teebox"} tee={tee} setNextPosition={setNextPosition} currentTee={props.currentTee} setRoutePos={setRoutePos}/><br/>
          <Buttons name={"Next Green"} style={{marginTop: "100em"}} validNextTee={validNextTee} isGo={isGo} setValidNextTee={setValidNextTee} validGreenTee={validGreenTee} setValidGreenTee={setValidGreenTee} setValidTarget={setValidTarget} tee={tee} setNextPosition={setNextPosition}/>
      </div>
      <Map routePos={routePos} currentPosition={currentPosition} nextPosition={nextPosition}/>
    </div>
  )
}

export default Main