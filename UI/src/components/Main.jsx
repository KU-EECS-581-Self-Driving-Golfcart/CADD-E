import React, {useState, useRef} from 'react'
import {tee1} from '../Data/tees';
import {pathOne, pathTwo, pathThree} from '../paths.js'
import { useLocation } from 'react-router-dom'
import Speedometer from './Speedometer.jsx';
import TeeCounter from './TeeCounter.jsx'
import Buttons from './Buttons.jsx';
import StopButton from './StopButton.jsx';
import Map from './Map.jsx';
import data from '../test_routeXY_T.json';
import GoButton from './GoButton.jsx';

const Main = () => {
  const tee = useLocation().state.state.tee;
  let startPos = [];
  if(tee === "black") startPos = tee1.blackPos
  if(tee === "bronze") startPos = tee1.bronzePos;
  if(tee === "silver") startPos = tee1.silverPos;
  if(tee === "gold") startPos = tee1.goldPos;

  const [isActive, setIsActive] = useState(false);
  const [validTarget, setValidTarget] = useState(false)
  const [targetTee, setTargetTee] = useState(2);
  const [currentTee, setCurrentTee] = useState(1);
  const [targetGreen, setTargetGreen] = useState(1);
  const timerId = useRef();
  const [routePos, setRoutePos] = useState([[0,0]])
  const [speed, setSpeed] = useState(0);
  const [currentPosition, setCurrentPosition] = useState(startPos)
  const [nextPosition, setNextPosition] = useState([0,0])
  const [validNextTee, setValidNextTee] = useState(true);
  const [validGreenTee, setValidGreenTee] = useState(true);
  const [isGo, setIsGo] = useState(false);

  let teeOrGreen = 0; //0 for tee, 1 for green;

  const startTimer = () => {
    clearInterval(timerId.current);
      let i = 0;
      timerId.current = setInterval(() => {
          setSpeed(speed => speed + 1);
          i +=1;
          if(i >= 20) clearInterval(timerId.current);
      }, 200)
  }

  const stopTimer = () => {
    clearInterval(timerId.current);
    let i = speed;
    timerId.current = setInterval(() => {
      setSpeed(speed => speed - 1);
      i -= 1;
      if(i === 0) clearInterval(timerId.current);
    }, 200)
}

function go() {
  handleSaveToPC(obj);
  if(!validTarget)
  {
    alert("Error!")
  }
  else
  {
    setIsGo(!isGo);
    setIsActive(!isActive)
    startTimer();
    if(targetTee<9) setTargetTee(targetTee+1)
    if(targetGreen<9) setTargetGreen(targetGreen+1)
    if(currentTee === 1) setRoutePos(data.test_routeXY);
    //if(currentTee === 1) setRoutePos(pathOne);
    else if(currentTee === 2) setRoutePos(pathTwo);
    else if(currentTee === 3) setRoutePos(pathThree);
    else 
    {
      let path = [currentPosition, nextPosition];
      console.log(path);
      (setRoutePos(path));
    }
  }

}

function stop() {
  setIsGo(!isGo)
  setValidTarget(!validTarget)
  stopTimer();
  setIsActive(!isActive)
  setCurrentPosition(nextPosition)
  setNextPosition([0,0])
  setRoutePos([[0,0]])
  if(currentTee<9) setCurrentTee(currentTee+1);
  setValidNextTee(true);
  setValidGreenTee(true);
}

  let obj = {
    teeBox: useLocation().state.state.tee,
    target: teeOrGreen
  }

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
    <div className='flex h-[100vh]'>
      <div className='w-[60%] h-[100vh] flex flex-col items-center justify-center'>
          <Speedometer speed={speed}/>
          <TeeCounter currentTee={currentTee} />
          <GoButton isActive={isActive} go={go}/>
          <StopButton isActive={isActive} stop={stop} />
          <Buttons validNextTee={validNextTee} isGo={isGo} setValidNextTee={setValidNextTee} setValidGreenTee={setValidGreenTee} setValidTarget={setValidTarget} name={"Next Teebox"} targetTee={targetTee} tee={tee} teeOrGreen={teeOrGreen} setNextPosition={setNextPosition}/><br/>
          <Buttons name={"Next Green"} style={{marginTop: "100em"}}/>
      </div>
      <Map routePos={routePos} currentPosition={currentPosition} targetTee={targetTee} nextPosition={nextPosition}/>
    </div>
  )
}
export default Main