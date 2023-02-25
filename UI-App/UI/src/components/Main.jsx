import React, {useState, useRef, useEffect } from 'react'
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
  //Initalize the starting positions for each different tee off location
  if(tee === "black") startPos = tee1.blackPos
  if(tee === "bronze") startPos = tee1.bronzePos;
  if(tee === "silver") startPos = tee1.silverPos;
  if(tee === "gold") startPos = tee1.goldPos;

  //App-wide states that are necessary to interact with all of the components
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
  const [stopClicked, setStopClicked] = useState(0);
  const [teeOrGreen, setTeeOrGreen] = useState(0);

  const [data, setData] = useState({}) //Used to inform target locations 

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
  handleSaveToPC(obj); //Download information about the intended route
  if(!validTarget)
  {
    alert("Error!") //Ensure that there is a valid target location
  }
  else
  {
    //Hide the Go button and display the stop button. Start the "speed" timer
    setIsGo(!isGo); 
    setIsActive(!isActive)
    startTimer();
    //Set the target location and set the route positions
    if(targetTee<9) setTargetTee(targetTee+1)
    if(targetGreen<9) setTargetGreen(targetGreen+1)
    //Causing bug for some reason ---------if(currentTee === 1) setRoutePos(data.test_routeXY);
    if(currentTee === 1) setRoutePos([[38.9825726,-95.2684392],[38.9825618,-95.2684386],[38.9825517,-95.2684382],[38.9825353,-95.2684375],[38.9825223,-95.2684372],[38.9825048,-95.2684359],[38.982488,-95.2684352],[38.982267885,-95.268422565000009],[38.98227947,-95.26842323000001],[38.982291055000005,-95.268423895],[38.98230264,-95.268424560000014],[38.982314225,-95.268425225],[38.982325810000006,-95.26842589],[38.982337395,-95.268426555],[38.98234898,-95.26842722],[38.982360565,-95.268427885],[38.98237215,-95.26842855000001],[38.982383735,-95.268429215000012],[38.982395319999995,-95.268429880000014],[38.982406905,-95.268430545],[38.98241849,-95.26843121],[38.982430074999996,-95.268431875],[38.98244166,-95.26843254],[38.982453244999995,-95.268433205000008],[38.98246483,-95.26843387],[38.982476414999994,-95.268434535],[38.9822563,-95.2684219],[38.9822385,-95.2684219],[38.9822204,-95.2684309],[38.9821825,-95.2684456],[38.9821581,-95.2684596],[38.9821336,-95.2684727],[38.982101,-95.2684899],[38.9820614,-95.2684971],[38.9820301,-95.2684879],[38.981991,-95.2684648],[38.9819616,-95.2684544],[38.9819318,-95.2684452],[38.9819124,-95.2684416],[38.9818946,-95.2684357],[38.9818484,-95.2684245],[38.981815,-95.2684129],[38.9817959,-95.2684065],[38.9817744,-95.2683994],[38.9817446,-95.2683858],[38.9817173,-95.2683715],[38.9816913,-95.2683615],[38.9816643,-95.2683563],[38.9816361,-95.2683535],[38.9816144,-95.2683511],[38.9815952,-95.2683491],[38.9815652,-95.2683467],[38.9815351,-95.2683431],[38.9814973,-95.2683372],[38.9814598,-95.2683272],[38.9814263,-95.2683156],[38.9813845,-95.2683017],[38.9813532,-95.2682901],[38.9813095,-95.2682738],[38.9812772,-95.2682622],[38.9812456,-95.2682467],[38.9812183,-95.2682343],[38.9811951,-95.2682235],[38.9811737,-95.2682164],[38.9811436,-95.268206],[38.9811136,-95.2681904],[38.9810863,-95.2681789],[38.9810603,-95.2681685],[38.9810336,-95.2681589],[38.9810079,-95.2681502],[38.9809797,-95.2681418],[38.9809524,-95.2681354],[38.9809257,-95.2681306],[38.9808994,-95.2681294],[38.9808718,-95.2681294],[38.9808449,-95.2681334],[38.9808256,-95.2681346],[38.9808027,-95.2681394],[38.980781,-95.2681426],[38.9807531,-95.2681474],[38.9807271,-95.2681482],[38.9807041,-95.2681474],[38.9806809,-95.2681482],[38.9806573,-95.2681482],[38.9806294,-95.2681462],[38.9806006,-95.2681446],[38.9805727,-95.2681438],[38.9805479,-95.268143],[38.9805324,-95.2681426],[38.980480854,-95.268149431],[38.980429308,-95.268156261999991],[38.9803762,-95.2681633],[38.9803499,-95.2681765],[38.9803263,-95.2681892],[38.9803021,-95.2682024],[38.9802767,-95.2682156],[38.9802507,-95.2682275],[38.980224,-95.2682399],[38.9801943,-95.2682522],[38.9801729,-95.268261],[38.9801456,-95.268271],[38.9801292,-95.2682778],[38.9801137,-95.2682845],[38.9800923,-95.2682945],[38.9800675,-95.2683057],[38.9800415,-95.2683168],[38.980017,-95.2683276],[38.9799885,-95.2683384],[38.9799643,-95.2683479],[38.9799504,-95.2683535],[38.9799277,-95.2683627],[38.9799101,-95.2683679],[38.979876,-95.2683786],[38.9798552,-95.2683846],[38.9798289,-95.2683934],[38.9798022,-95.2684022],[38.9797898,-95.2684061],[38.9797759,-95.2684125],[38.9797504,-95.2684241],[38.9797244,-95.2684329],[38.9796971,-95.2684412],[38.9796726,-95.268446],[38.9796469,-95.2684516],[38.97962,-95.2684576],[38.9795952,-95.2684644],[38.9795679,-95.2684739],[38.9795434,-95.2684851],[38.9795229,-95.2684978],[38.9795047,-95.268511],[38.9794826,-95.2685282],[38.9794637,-95.2685445],[38.9794417,-95.268562],[38.9794197,-95.2685836],[38.9794083,-95.2685947],[38.9793912,-95.2686075],[38.9793776,-95.2686179],[38.9793611,-95.268627],[38.979346,-95.2686338],[38.979328,-95.2686406],[38.9793106,-95.268647],[38.9792868,-95.2686549],[38.9792607,-95.2686625],[38.9792496,-95.2686689],[38.9792378,-95.2686765],[38.9792257,-95.2686833],[38.9792155,-95.2686928],[38.9791956,-95.2687151],[38.9791808,-95.2687435],[38.9791687,-95.2687718],[38.9791547,-95.2688033],[38.9791411,-95.2688332],[38.9791256,-95.2688631],[38.9791073,-95.2688922],[38.9790964,-95.2689053],[38.9790834,-95.2689149],[38.9790552,-95.2689229],[38.9790283,-95.2689281],[38.9790041,-95.2689328],[38.9789759,-95.2689392],[38.9789533,-95.268944],[38.9789263,-95.268952],[38.9789009,-95.268962],[38.9788745,-95.2689755],[38.9788658,-95.2689811],[38.9788264,-95.2690085],[38.9787974,-95.2690288],[38.9787768,-95.2690401],[38.9787606,-95.2690418],[38.9787431,-95.2690316],[38.9787299,-95.2690153],[38.9787225,-95.268991],[38.9787146,-95.2689504],[38.9787106,-95.2689093],[38.9787067,-95.2688692],[38.9787036,-95.2688298],[38.978701,-95.2687931],[38.9786993,-95.2687593],[38.9786962,-95.2687238],[38.9786944,-95.2686922],[38.9786922,-95.2686499],[38.9786892,-95.2686104],[38.9786874,-95.2685743],[38.9786861,-95.2685416],[38.9786857,-95.2685055],[38.9786848,-95.2684695],[38.9786848,-95.2684029],[38.9786848,-95.2683668],[38.9786848,-95.2683308],[38.9786848,-95.2682947],[38.9786857,-95.2682608],[38.9786848,-95.2682242],[38.9786835,-95.2681881],[38.9786826,-95.2681514],[38.9786839,-95.2681137],[38.9786839,-95.2680776],[38.9786839,-95.2680409],[38.9786843,-95.2680043],[38.9786861,-95.267971],[38.9786865,-95.267936],[38.978687,-95.2679022],[38.9786865,-95.2678639],[38.9786865,-95.2678278],[38.9786874,-95.2677838],[38.9786892,-95.267719],[38.97869,-95.2676863],[38.9786905,-95.267649],[38.9786909,-95.2676141],[38.9786917,-95.2675691],[38.9786917,-95.26750335],[38.9786917,-95.2674376],[38.9786917,-95.26737185],[38.9787237,-95.2673061],[38.9787345,-95.2672972],[38.9787456,-95.2672876],[38.9787604,-95.2672755],[38.9787958,-95.2672309],[38.9788135,-95.2671921],[38.9788238,-95.2671541],[38.9788275,-95.2671124],[38.978826,-95.2670659],[38.9788231,-95.26701],[38.9788223,-95.2669645],[38.9788187,-95.2669095],[38.9788144,-95.2668681],[38.9788092,-95.2668111],[38.9788019,-95.2667648],[38.9787925,-95.2667159],[38.9787826,-95.2666836],[38.9787768,-95.2666632],[38.978773,-95.2666495],[38.9787487,-95.2665879],[38.9787273,-95.2665395],[38.9787007,-95.266475],[38.9786683,-95.2664067],[38.9786528,-95.2663574],[38.9786381,-95.2663176],[38.9786204,-95.2662721],[38.9786071,-95.2662294],[38.9785968,-95.2661971],[38.9785909,-95.266163],[38.9785879,-95.2661156],[38.978585,-95.2660559],[38.978582,-95.2659961],[38.9785806,-95.2659544],[38.9785776,-95.2659193],[38.9785717,-95.2658757],[38.9785621,-95.2658121],[38.9785496,-95.2657552],[38.9785356,-95.2656927],[38.9785157,-95.2656149],[38.978484,-95.2655144],[38.978456,-95.26543],[38.9784331,-95.2653617],[38.978414,-95.2653105],[38.9783963,-95.2652707],[38.9783778,-95.2652299],[38.9783329,-95.2651616],[38.9782842,-95.2650848],[38.9782437,-95.2650326],[38.9782193,-95.2649871],[38.9781729,-95.2649122],[38.9781427,-95.2648686],[38.9781132,-95.2648278],[38.9780808,-95.2647899],[38.9780395,-95.2647434],[38.9780004,-95.2647036],[38.9779562,-95.2646647],[38.977906,-95.2646192],[38.9778308,-95.2645547],[38.9778146,-95.2645357],[38.9778021,-95.2645168],[38.9777918,-95.2644949],[38.9777778,-95.264458],[38.9777652,-95.2644143],[38.9777534,-95.2643755],[38.9777446,-95.2643347],[38.977749,-95.2643043],[38.9777638,-95.2642749],[38.9777866,-95.2642408],[38.9778073,-95.2642076],[38.9778272,-95.2641744],[38.9778434,-95.2641431],[38.9778544,-95.2641242],[38.9778684,-95.2640976],[38.9778824,-95.2640682],[38.9778942,-95.2640255],[38.9778994,-95.263999],[38.9778979,-95.2639724],[38.977892,-95.2639392],[38.9778773,-95.2638966],[38.9778707,-95.2638691],[38.9778714,-95.2638463],[38.9778773,-95.2638169],[38.9778787,-95.2637994],[38.9778787,-95.2637896],[38.9778778,-95.2637808],[38.9778767,-95.2637726],[38.977864175000008,-95.26374295],[38.977851650000005,-95.2637133],[38.977839125,-95.26368365],[38.9778266,-95.263654],[38.9778226,-95.2636478],[38.9778102,-95.2636291],[38.9777918,-95.2635874],[38.9777756,-95.2635372],[38.9777571,-95.2634869],[38.9777387,-95.263449],[38.977707,-95.2633845],[38.9777046,-95.2633796]]);
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

//Function handler for when the stop button is pressed
function stop() {
  //Hide the stop button and display the go button once again.
  setIsGo(!isGo)
  setValidTarget(!validTarget)
  stopTimer();
  fetch("/StopCommand").then(
    res => res.json()
).then(
  data => {
    setData(data);
    console.log(data);
  }
)
  setIsActive(!isActive)
  setStopClicked(stopClicked++);
  //Set the next positions and route
  setCurrentPosition(nextPosition)
  setNextPosition([0,0])
  setRoutePos([[0,0]])
  if(currentTee<9) setCurrentTee(currentTee+1);

  //Modify these boolean variables so that they can be selected once again
  setValidNextTee(true);
  setValidGreenTee(true);
}


  //Obj information that is used to communicate route intentions 
  let obj = {
    teeBox: useLocation().state.state.tee,
    target: teeOrGreen
  }

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
          <TeeCounter currentTee={currentTee} />
          <GoButton isActive={isActive} go={go}/>
          <StopButton isActive={isActive} stop={stop} />
          <Buttons validNextTee={validNextTee} isGo={isGo} setValidNextTee={setValidNextTee} setValidGreenTee={setValidGreenTee} setValidTarget={setValidTarget} name={"Next Teebox"} targetTee={targetTee} tee={tee} setTeeOrGreen={setTeeOrGreen} setNextPosition={setNextPosition}/><br/>
          <Buttons name={"Next Green"} style={{marginTop: "100em"}}/>
      </div>
      <Map routePos={routePos} currentPosition={currentPosition} targetTee={targetTee} nextPosition={nextPosition}/>
    </div>
  )
}
export default Main