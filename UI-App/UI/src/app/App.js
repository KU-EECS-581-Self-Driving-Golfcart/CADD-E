import React from 'react'
import Main from './Main';
import Welcome from '../components/Welcome';
import { BrowserRouter, Routes, Route } from 'react-router-dom'

function App(props) {
  // var ros = new ROSLIB.Ros({
  //   url : 'ws://localhost:9090'
  // });

  // ros.on('connection', function() {
  //   console.log('Connected to websocket server.');
  // });

  // ros.on('error', function(error) {
  //   console.log('Error connecting to websocket server: ', error);
  // });

  // ros.on('close', function() {
  //   console.log('Connection to websocket server closed.');
  // });

  return (
    <div>
    <BrowserRouter>
      <Routes>
        <Route path="/" element={<Welcome />} />
        <Route path="/main" element={
          <Main 
            state={props.state}
            dispatch={props.dispatch}
          />
        }/>
       </Routes>
    </BrowserRouter>
    </div>
  );
}

export default App;
