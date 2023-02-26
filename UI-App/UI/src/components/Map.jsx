import React, {useState} from 'react';
import { MapContainer, TileLayer, Marker, Popup, useMapEvents, Polyline} from 'react-leaflet';
import icon from '../Images/icon.svg';
import L from 'leaflet';

//Map component displays the right-hand side map and will have route and target locations displayed. 
const Map = (props) => {

    //When the map is clicked, this function will provide the current location of the vehicle. 
    //Eventually, this will be used with the GPS to provide accurate data. 
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

    //Set the map's current location icon
    const myIcon = new L.Icon({
        iconUrl: icon,
        iconRetinaUrl: icon,
        popupAnchor:  [-0, -0],
        iconSize: [20,25],     
    });

  return (
    //Render the Map components with the corresponding Markers and route information as needed
    <div className='bg-[#3E3B3B] pl-6 py-7 rounded-l-xl w-[40%]'>
    {/* <iframe title="map" className='map rounded-l-xl' width="500" height="780" frameborder="0" scrolling="no" marginheight="0" marginwidth="0" src="https://www.openstreetmap.org/export/embed.html?bbox=-95.26860415935516%2C38.976263117537805%2C-95.26184499263765%2C38.97992040605664&amp;layer=mapnik&amp;marker=38.978091785409354%2C-95.2652245759964"></iframe> */}
    <MapContainer style={{height: 950, width:600}} center = {[38.979, -95.266]} zoom = {16}>
       <TileLayer
         url="https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png"
         attribution='&copy; <a href="https://osm.org/copyright">OpenStreetMap</a> 
         contributors'
       />
       <Polyline color="red" positions={props.routePos}/>
       <Marker position={props.currentPosition} icon={myIcon}>
         <Popup>
           Hole {props.targetTee}
         </Popup>
       </Marker>
      
       <Marker position={props.nextPosition}>
         <Popup>
           Hole {props.targetTee+1}
         </Popup>
       </Marker>
       <LocationMarker/>
     </MapContainer>         
 </div>
  )
}

export default Map