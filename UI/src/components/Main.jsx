import React, {useState, useRef} from 'react'
import {pathOne, pathTwo, pathThree} from '../paths.js'
import { Icon } from '@iconify/react';
import icon from '../Images/icon.svg'
import { MapContainer, TileLayer, Marker, Popup, useMapEvents, Polyline} from 'react-leaflet';
import L from 'leaflet';
import 'leaflet-routing-machine'
import 'leaflet-routing-machine/dist/leaflet-routing-machine.css'
import { useLocation } from 'react-router-dom'
import data from '../test_routeXY_T.json';

// import { MapContainer } from 'react-leaflet';
// import { TileLayer } from 'leaflet';
// import { Marker } from 'react-leaflet';
// import { Popup } from 'react-leaflet';
// import { useMap } from 'react-leaflet';
const Main = () => {

  const tee1 = {
    blackPos: [38.9776672, -95.2634994],
    bronzePos: [38.97771, -95.26372],
    silverPos: [38.97777, -95.26396],
    goldPos: [38.97785, -95.26433],
    greenPos: [38.97858, -95.26700]
  }

  const tee2 = {
    blackPos: [38.9786234, -95.2675139],
    bronzePos: [38.97861,-95.26771],
    silverPos: [38.97860,-95.26792],
    goldPos: [38.97860,-95.26810],
    greenPos: [38.97853,-95.26923]
  }

  const tee3 = {
    blackPos: [38.9788485, -95.2688737],
    bronzePos: [38.97853,-95.26923],
    silverPos: [38.97965,-95.26857],
    goldPos: [38.97965,-95.26857],
    greenPos: [38.98292,-95.26895]
  }

  const tee4 = {
    blackPos: [38.9829038, -95.2683226],
    bronzePos: [38.98272,-95.26826],
    silverPos: [38.98257,-95.26823],
    goldPos: [38.98179,-95.26784],
    greenPos: [38.97945,-95.26793]
  }

  const tee5 = {
    blackPos: [38.9788209, -95.2676372],
    bronzePos: [38.97945,-95.26793],
    silverPos: [38.97913,-95.26746],
    goldPos: [38.97933,-95.26722],
    greenPos: [38.98179,-95.26600]
  }

  const tee6 = {
    blackPos: [38.9819392, -95.2653599],
    bronzePos: [38.98163,-95.26515],
    silverPos: [38.98140,-95.26503],
    goldPos: [38.98118,-95.26490],
    greenPos: [38.97901,-95.26372]
  }

  const tee7 = {
    blackPos: [38.9780217, -95.2634658],
    bronzePos: [38.97809,-95.26325],
    silverPos: [38.97815,-95.26301],
    goldPos: [38.97821,-95.26279],
    greenPos: [38.97857,-95.26146]
  }

  const tee8 = {
    blackPos: [38.9787367, -95.2607026],
    bronzePos: [38.97857,-95.26077],
    silverPos: [38.97839,-95.26084],
    goldPos: [38.97794,-95.26109],
    greenPos: [38.97547,-95.26184]
  }

  const tee9 = {
    blackPos: [38.9752793, -95.2623357],
    bronzePos: [38.97549,-95.26235],
    silverPos: [38.97564,-95.26245],
    goldPos: [38.97589,-95.26246],
    greenPos: [38.97794,-95.26232]
  }

  const tee = useLocation().state.state.tee;

  let startPos = [];
  if(tee === "black") startPos = tee1.blackPos;
  if(tee === "bronze") startPos = tee1.bronzePos;
  if(tee === "silver") startPos = tee1.silverPos;
  if(tee === "gold") startPos = tee1.goldPos;


  const [isActive, setIsActive] = useState(false);
  const [validTarget, setValidTarget] = useState(false)
  const [targetTee, setTargetTee] = useState(2);
  const [currentTee, setCurrentTee] = useState(1);
  const [targetGreen, setTargetGreen] = useState(1);
  const timerId = useRef();
  const [speed, setSpeed] = useState(0);
  const [currentPosition, setCurrentPosition] = useState(startPos)
  const [nextPosition, setNextPosition] = useState([0,0])
  const [routePos, setRoutePos] = useState([[0,0]])
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

  const myIcon = new L.Icon({
    iconUrl: icon,
    iconRetinaUrl: icon,
    popupAnchor:  [-0, -0],
    iconSize: [20,25],     
});

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

function incTee() {
  if(validNextTee && !isGo)
  {
    setValidNextTee(false);
    setValidGreenTee(true);
    setValidTarget(true)
    console.log(targetTee)
    if(targetTee===2) {
      if(tee === "black") setNextPosition(tee2.blackPos);
      if(tee === "bronze") setNextPosition(tee2.bronzePos);
      if(tee === "silver") setNextPosition(tee2.silverPos);
      if(tee === "gold") setNextPosition(tee2.goldPos);
    }
    if(targetTee===3) {
      if(tee === "black") setNextPosition(tee3.blackPos);
      if(tee === "bronze") setNextPosition(tee3.bronzePos);
      if(tee === "silver") setNextPosition(tee3.silverPos);
      if(tee === "gold") setNextPosition(tee3.goldPos);
    }
    if(targetTee===4) {
      if(tee === "black") setNextPosition(tee4.blackPos);
      if(tee === "bronze") setNextPosition(tee4.bronzePos);
      if(tee === "silver") setNextPosition(tee4.silverPos);
      if(tee === "gold") setNextPosition(tee4.goldPos);
    }
    if(targetTee===5) {
      if(tee === "black") setNextPosition(tee5.blackPos);
      if(tee === "bronze") setNextPosition(tee5.bronzePos);
      if(tee === "silver") setNextPosition(tee5.silverPos);
      if(tee === "gold") setNextPosition(tee5.goldPos);
    }
    if(targetTee===6) {
      if(tee === "black") setNextPosition(tee6.blackPos);
      if(tee === "bronze") setNextPosition(tee6.bronzePos);
      if(tee === "silver") setNextPosition(tee6.silverPos);
      if(tee === "gold") setNextPosition(tee6.goldPos);
    }
    if(targetTee===7) {
      if(tee === "black") setNextPosition(tee7.blackPos);
      if(tee === "bronze") setNextPosition(tee7.bronzePos);
      if(tee === "silver") setNextPosition(tee7.silverPos);
      if(tee === "gold") setNextPosition(tee7.goldPos);
    }
    if(targetTee===8) {
      if(tee === "black") setNextPosition(tee8.blackPos);
      if(tee === "bronze") setNextPosition(tee8.bronzePos);
      if(tee === "silver") setNextPosition(tee8.silverPos);
      if(tee === "gold") setNextPosition(tee8.goldPos);
    }
    if(targetTee===9) {
      if(tee === "black") setNextPosition(tee9.blackPos);
      if(tee === "bronze") setNextPosition(tee9.bronzePos);
      if(tee === "silver") setNextPosition(tee9.silverPos);
      if(tee === "gold") setNextPosition(tee9.goldPos);
    }
    teeOrGreen = 0;
  }
}

function incGreen() {
  if(validGreenTee && !isGo)
  {
    setValidGreenTee(false);
    setValidNextTee(true);
    setValidTarget(true)
    console.log(targetGreen)
    if(targetGreen===1) setNextPosition(tee1.greenPos);
    if(targetGreen===2) setNextPosition(tee2.greenPos);
    if(targetGreen===3) setNextPosition(tee3.greenPos);
    if(targetGreen===4) setNextPosition(tee4.greenPos);
    if(targetGreen===5) setNextPosition(tee5.greenPos);
    if(targetGreen===6) setNextPosition(tee6.greenPos);
    if(targetGreen===7) setNextPosition(tee7.greenPos);
    if(targetGreen===8) setNextPosition(tee8.greenPos);
    if(targetGreen===8) setNextPosition(tee9.greenPos);
  }
  teeOrGreen = 1;
}


  function LocationMarker() {
    const [position, setPosition] = useState(null)
    const map = useMapEvents({
      click() {
        map.locate()
      },
      locationfound(e) {
        setPosition(e.latlng)
        map.flyTo(e.latlng, map.getZoom())
      },
    })
  
    return position === null ? null : (
      <Marker position={position} icon={myIcon}>
        <Popup>Current Location</Popup>
      </Marker>
    )
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
          <div className='flex flex-row justify-center border-[#3E3B3B] border-[3px] drop-shadow-2xl shadow-2xl border-solid rounded-xl py-16 mt-12 w-[350px] mt-[-1em]'>
            <h1 className='text-[#3E3B3B] text-5xl'>{speed}</h1>
            <h1 className='text-[#3E3B3B] text-5xl ml-2'>mph</h1>
          </div>
          <div className='flex text-6xl justify-center bg-[#3E3B3B] text-white w-[200px] py-2 rounded-b-2xl shadow-xl'>
            <Icon icon="game-icons:golf-tee" className=''/>
            <h2 className='ml-2 tee'>{currentTee}</h2>
          </div>
          <button style={{display: isActive ? 'none' : 'block'}} className='my-10' onClick={go}>
            <div className="flex bg-green-700 aspect-square shrink-0 rounded-full grow-0 w-[260px] h-[260px] justify-center items-center flex-col shadow-2xl drop-shadow-2xl">
            <Icon icon="mdi:golf-cart" className='text-white text-9xl mt-[1rem]'/>
            <Icon icon="file-icons:go" className='text-white text-9xl mt-[-.35em] mr-[.2em]'/>
            </div>
          </button>
          <button style={{display: isActive ? 'block' : 'none'}} onClick={stop} className='my-6 hidden'>
              <Icon icon="game-icons:stop-sign"  className='text-[275px] text-red-700 border-[#3E3B3B] drop-shadow-2xl'/>
          </button>
          <button onClick={incTee} className='bg-[#3E3B3B] text-white py-4 w-[275px] rounded-xl shadow-lg text-3xl'>Next Teebox</button>
          <button onClick={incGreen} className='bg-[#3E3B3B] text-white py-4 w-[275px] rounded-xl shadow-lg mt-4 text-3xl'>Next Green</button>
      </div>


      
      <div className='bg-[#3E3B3B] pl-6 py-7 rounded-l-xl w-[40%]'>
         {/* <iframe title="map" className='map rounded-l-xl' width="500" height="780" frameborder="0" scrolling="no" marginheight="0" marginwidth="0" src="https://www.openstreetmap.org/export/embed.html?bbox=-95.26860415935516%2C38.976263117537805%2C-95.26184499263765%2C38.97992040605664&amp;layer=mapnik&amp;marker=38.978091785409354%2C-95.2652245759964"></iframe> */}
         <MapContainer style={{height: 950, width:600}} center = {[38.979, -95.266]} zoom = {16}>
            <TileLayer
              url="https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png"
              attribution='&copy; <a href="https://osm.org/copyright">OpenStreetMap</a> 
              contributors'
            />
            <Polyline color="red" positions={routePos}/>
            <Marker position={currentPosition} icon={myIcon}>
              <Popup>
                Hole {targetTee}
              </Popup>
            </Marker>
            
            <Marker position={nextPosition}>
              <Popup>
                Hole {targetTee+1}
              </Popup>
            </Marker>
            <LocationMarker/>
          </MapContainer>         
      </div>
    </div>

    
  )
}

export default Main