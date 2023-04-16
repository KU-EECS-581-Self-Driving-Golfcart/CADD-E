import React from 'react'
import Main from './Main';
import Welcome from '../components/Welcome';
import { BrowserRouter, Routes, Route } from 'react-router-dom'
import ROSLIB from 'roslib'
export let teePub;

function App(props) {
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

  var listenerGPS = new ROSLIB.Topic({
    ros : ros,
    name : '/gps',
    messageType : 'cadd_e_interface/msg/GPS'
  }); 

  return (
    <div>
    <BrowserRouter>
      <Routes>
        <Route path="/" element={<Welcome />} />
        <Route path="/main" element={
          <Main 
            state={props.state}
            dispatch={props.dispatch}
            teePub={teePub} goPub={goPub} listener={listener} listenerGPS={listenerGPS}
          />
        }/>
       </Routes>
    </BrowserRouter>
    </div>
  );
}

export default App;
