import React, {useState, useRef} from 'react'
import { Icon } from '@iconify/react';
import icon from '../Images/icon.svg'
import { MapContainer, TileLayer, Marker, Popup, useMapEvents, Polygon} from 'react-leaflet';
import L from 'leaflet';
import 'leaflet-routing-machine'
import 'leaflet-routing-machine/dist/leaflet-routing-machine.css'
import RoutingControl from './RoutingControl'
// import { MapContainer } from 'react-leaflet';
// import { TileLayer } from 'leaflet';
// import { Marker } from 'react-leaflet';
// import { Popup } from 'react-leaflet';
// import { useMap } from 'react-leaflet';
const Main = () => {


  const [count, setCount] = useState(1);
  const [counter, setCounter] = useState(1);
  const [routeCount, setrouteCount] = useState(0);
  const [isActive, setIsActive] = useState(false);
  const timerId = useRef();
  const [speed, setSpeed] = useState(0);
  let position1 = []
  let position2 = []


  let position3 = [
    [0,0]
  ];

  const startTimer = () => {
    clearInterval(timerId.current);
      let i = 0;
      timerId.current = setInterval(() => {
          setSpeed(speed => speed + 1);
          i +=1;
          if(i >= 20) clearInterval(timerId.current);
      }, 200)
  }

  if(routeCount === 1) {
    position3 = [
      [38.9776672, -95.2634994],
      [38.9786234, -95.2675139]
    ];
  }
  if((routeCount === 2) || (routeCount === 4) || (routeCount === 6) || (routeCount === 8) || (routeCount === 10) || (routeCount === 12) || (routeCount === 14) || (routeCount === 16) || (routeCount === 18)) {
    position3 = [
        [0,0]
    ];
  }
  if(routeCount === 3) {
    position3 = [
      [38.9786234, -95.2675139],
      [38.9788485, -95.2688737]
    ];
  }
  if(routeCount === 5) {
    position3 = [
      [38.9788485, -95.2688737],
      [38.9829038, -95.2683226]
    ];
  }
  if(routeCount === 7) {
    position3 = [
      [38.9829038, -95.2683226],
      [38.9788209, -95.2676372]
    ];
  }
  if(routeCount === 9) {
    position3 = [
      [38.9788209, -95.2676372],
      [38.9819392, -95.2653599]
    ];
  }
  if(routeCount === 11) {
    position3 = [
      [38.9819392, -95.2653599],
      [38.9780217, -95.2634658]
    ];
  }
  if(routeCount === 13) {
    position3 = [
      [38.9780217, -95.2634658],
      [38.9787367, -95.2607026]
    ];
  }
  if(routeCount === 15) {
    position3 = [
      [38.9787367, -95.2607026],
      [38.9752793, -95.2623357]
    ];
  }
  if(routeCount === 17) {
    position3 = [
      [38.9752793, -95.2623357],
      [38.9752793, -95.2623357]
    ];
  }
  
  if(count===1) {
    position1 = [38.9776672, -95.2634994];
    position2 = [38.9786234, -95.2675139];
  }
  if(count==2) {
    position1 = [38.9786234, -95.2675139];
    position2 = [0,0];
  }
  if(count===3) {
    position1 = [38.9786234, -95.2675139];
    position2 = [38.9788485, -95.2688737];
  }
  if(count==4) {
    position1 = [38.9788485, -95.2688737];
    position2 = [0,0];
  }
  if(count===5) {
    position1 = [38.9788485, -95.2688737];
    position2 = [38.9829038, -95.2683226];
  }
  if(count==6) {
    position1 = [38.9829038, -95.2683226];
    position2 = [0,0];
  }
  if(count===7) {
    position1 = [38.9829038, -95.2683226];
    position2 = [38.9788209, -95.2676372];
  }
  if(count==8) {
    position1 = [38.9788209, -95.2676372];
    position2 = [0,0];
  }
  if(count===9) {
    position1 = [38.9788209, -95.2676372];
    position2 = [38.9819392, -95.2653599];
  }
  if(count==10) {
    position1 = [38.9819392, -95.2653599];
    position2 = [0,0];
  }
  if(count===11) {
    position1 = [38.9819392, -95.2653599];
    position2 = [38.9780217, -95.2634658];
  }
  if(count==12) {
    position1 = [38.9780217, -95.2634658];
    position2 = [0,0];
  }
  if(count===13) {
    position1 = [38.9780217, -95.2634658];
    position2 = [38.9787367, -95.2607026];
  }
  if(count==14) {
    position1 = [38.9787367, -95.2607026];
    position2 = [0,0];
  }
  if(count===15) {
    position1 = [38.9787367, -95.2607026];
    position2 = [38.9752793, -95.2623357];
  }
  if(count==16) {
    position1 = [38.9752793, -95.2623357];
    position2 = [0,0];
  }
  if(count===17) {
    position1 = [38.9752793, -95.2623357];
    position2 = [38.9752793, -95.2623357];
  }
  if(count==18) {
    position1 = [38.9752793, -95.2623357];
    position2 = [0,0];
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
  handleClick();
  startTimer();
  goInc();
  console.log(routeCount);
}

function goInc() {
  if(routeCount<18) setrouteCount(routeCount+1);
}

function stop() {
  handleClick();
  stopTimer();
  goInc();
  console.log(routeCount);
  if(count<18) setCount(count+1);
}

  const handleClick = () => {
    setIsActive(current => !current);
  };

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
  
  return (
    <div className='flex h-[100vh]'>
      <div className='w-[60%] h-[100vh] flex flex-col items-center'>
          <div className='flex flex-row justify-center border-[#3E3B3B] border-[3px] drop-shadow-2xl shadow-2xl border-solid rounded-xl py-16 mt-12 w-[350px]'>
            <h1 className='text-[#3E3B3B] text-5xl'>{speed}</h1>
            <h1 className='text-[#3E3B3B] text-5xl ml-2'>mph</h1>
          </div>
          <div className='flex text-6xl justify-center bg-[#3E3B3B] text-white w-[200px] py-2 rounded-b-2xl shadow-xl'>
            <Icon icon="game-icons:golf-tee" className=''/>
            <h2 className='ml-2 tee'>{counter}</h2>
          </div>
          <button style={{display: isActive ? 'none' : 'block'}} className='my-6' onClick={go}>
            <div class="flex bg-green-700 aspect-square shrink-0 rounded-full grow-0 w-[260px] h-[260px] justify-center items-center flex-col shadow-2xl drop-shadow-2xl">
            <Icon icon="mdi:golf-cart" className='text-white text-9xl mt-[1rem]'/>
            <Icon icon="file-icons:go" className='text-white text-9xl mt-[-.35em] mr-[.2em]'/>
            </div>
          </button>
          <button style={{display: isActive ? 'block' : 'none'}} onClick={stop} className='my-6 hidden'>
              <Icon icon="game-icons:stop-sign"  className='text-[275px] text-red-700 border-[#3E3B3B] drop-shadow-2xl'/>
          </button>
          <button onClick={function incTee() {if(count<18) setCount(count+1); if(counter<9) setCounter(counter+1)}} className='bg-[#3E3B3B] text-white py-4 w-[275px] rounded-xl shadow-lg text-3xl'>Next Teebox</button>
          <button className='bg-[#3E3B3B] text-white py-4 w-[275px] rounded-xl shadow-lg mt-4 text-3xl'>Next Green</button>
      </div>


      
      <div className='bg-[#3E3B3B] pl-4 py-3 rounded-l-xl'>
         {/* <iframe title="map" className='map rounded-l-xl' width="500" height="780" frameborder="0" scrolling="no" marginheight="0" marginwidth="0" src="https://www.openstreetmap.org/export/embed.html?bbox=-95.26860415935516%2C38.976263117537805%2C-95.26184499263765%2C38.97992040605664&amp;layer=mapnik&amp;marker=38.978091785409354%2C-95.2652245759964"></iframe> */}
         <MapContainer style={{height: 780, width: 500}} center = {[38.979, -95.266]} zoom = {16}>
            <TileLayer
              url="https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png"
              attribution='&copy; <a href="https://osm.org/copyright">OpenStreetMap</a> 
              contributors'
            />
            <Polygon color="red" positions={position3}/>
            <Marker position={position1} icon={myIcon}>
              <Popup>
                Hole {count}
              </Popup>
            </Marker>
            
            <Marker position={position2}>
              <Popup>
                Hole 2
              </Popup>
            </Marker>
            <LocationMarker/>
          </MapContainer>         
      </div>
    </div>

    
  )
}

export default Main