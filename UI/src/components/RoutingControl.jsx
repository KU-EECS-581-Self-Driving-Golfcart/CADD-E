import L from "leaflet";
import { createControlComponent } from "@react-leaflet/core";
import "leaflet-routing-machine";

const createRoutineMachineLayer = (props) => {
  
    const instance = L.Routing.control({
      position: 'topleft', // Where to place control on the map
      waypoints: [
        L.latLng(38.9776672, -95.2634994),
        L.latLng(38.9786234, -95.2675139)
      ],
      lineOptions: { // Options for the routing line
        styles: [
          {
            color: '#757de8',
          },
        ],
      },
    });
  
    return instance;
  };

// Pass our createRoutingMachineLayer to the createControlHook:
const RoutingMachine = createControlComponent(createRoutineMachineLayer);

// Export
export default RoutingMachine;